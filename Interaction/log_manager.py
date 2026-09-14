import copy
import collections
import json
import os
import re
import subprocess
import threading
from dataclasses import dataclass
from types import MappingProxyType
from typing import Callable, Mapping

from Interaction.Kalman_Filter import VelocityKalmanFilter
from Interaction.live_logger import LiveLogger
from log_manager_abs import LogManager
from cflib.crazyflie.log import LogConfig
import time
import logging

logger = logging.getLogger(__name__)


CF_TIMESTAMP_MODULUS_MS = 1 << 24
CONTACT_SOURCE_TIMESTAMP_BASIS = (
    'firmware_latched_stabilizer_tick_low16_v1'
)
CONTACT_SOURCE_MAX_TRANSPORT_SKEW_MS = 100
_CONTACT_SOURCE_EPOCH_KEYS = {
    # The packed live stream is first. Legacy keys remain readable so old
    # flight logs and focused compatibility fixtures can still be replayed.
    'GYRO_1KHZ': ('contactImu.epoch', 'contactGyro.epoch'),
    'ACC_ALIGN': ('contactAccel.epoch',),
    'CONTACT_STATE_SEED': ('contactSeed.epoch',),
}


def reconstruct_contact_source_timestamp_ms(transport_timestamp_ms, epoch):
    """Lift a firmware low-16 source tick onto the nearest CRTP 24-bit tick.

    The transport header supplies the high-order neighborhood while the
    producer-latched ``epoch`` supplies the exact low 16 bits.  Returning a
    value only for a small transport/source skew avoids silently accepting the
    wrong 65.536 s epoch after a delayed or malformed packet.
    """
    if isinstance(transport_timestamp_ms, bool) or isinstance(epoch, bool):
        return None, None
    try:
        transport = int(transport_timestamp_ms)
        low16 = int(epoch)
    except (TypeError, ValueError, OverflowError):
        return None, None
    if not 0 <= transport < CF_TIMESTAMP_MODULUS_MS:
        return None, None
    if not 0 <= low16 < (1 << 16):
        return None, None
    base = (transport & ~0xFFFF) | low16
    candidates = tuple({
        base % CF_TIMESTAMP_MODULUS_MS,
        (base - (1 << 16)) % CF_TIMESTAMP_MODULUS_MS,
        (base + (1 << 16)) % CF_TIMESTAMP_MODULUS_MS,
    })

    def signed_delta(left, right):
        delta = (int(left) - int(right)) % CF_TIMESTAMP_MODULUS_MS
        if delta >= CF_TIMESTAMP_MODULUS_MS // 2:
            delta -= CF_TIMESTAMP_MODULUS_MS
        return delta

    source = min(
        candidates,
        key=lambda candidate: abs(signed_delta(transport, candidate)),
    )
    transport_minus_source_ms = signed_delta(transport, source)
    if abs(transport_minus_source_ms) > (
            CONTACT_SOURCE_MAX_TRANSPORT_SKEW_MS):
        return None, transport_minus_source_ms
    return int(source), int(transport_minus_source_ms)


def _immutable_copy(value):
    """Recursively copy JSON-like diagnostics into immutable containers."""
    if isinstance(value, dict):
        return MappingProxyType({
            key: _immutable_copy(item) for key, item in value.items()
        })
    if isinstance(value, (list, tuple)):
        return tuple(_immutable_copy(item) for item in value)
    return copy.deepcopy(value)


@dataclass(frozen=True)
class CfLogPacket:
    """Immutable, read-only view delivered to shadow packet listeners."""

    sequence: int
    group: str
    cf_timestamp_ms: int
    host_receive_time_s: float
    data: Mapping[str, object]
    host_receive_monotonic_s: float | None = None
    # For ordinary log blocks cf_timestamp_ms is the transport header tick.
    # Contact telemetry instead replaces it with the producer-latched
    # stabilizer tick reconstructed from a low-16 epoch field.
    transport_cf_timestamp_ms: int | None = None
    source_cf_timestamp_basis: str | None = None
    source_snapshot_atomic: bool = False


@dataclass(frozen=True)
class MocapFramePacket:
    """Immutable copy of the primary Vicon frame seen after forwarding."""

    sequence: int
    group: str
    host_receive_time_s: float
    data: Mapping[str, object]
    # Real Vicon frames currently have no Crazyflie clock.  CrazySim and
    # offline fixtures may provide one explicitly; consumers must never invent
    # a device timestamp from host arrival order.
    cf_timestamp_ms: int | None = None
    host_receive_monotonic_s: float | None = None


class InteractionLogger(LogManager):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.cf_var_logger = None
        self.cf_log_times = []
        self.cf_log_data = None
        self.cf_log_group_times = {}
        self.cf_log_group_packets = collections.defaultdict(
            lambda: collections.deque(maxlen=1000)
        )
        self.cf_log_packet_lock = threading.Lock()
        # cflib can deliver a packet that was already queued after a log block
        # has been stopped.  Serialize callback completion with shutdown so the
        # LiveLogger is not closed while one of those callbacks is still
        # writing, and reject callbacks that arrive after shutdown begins.
        self.cf_log_callback_lock = threading.Lock()
        self._accepting_cf_log_callbacks = True
        self._cf_log_packet_sequence = 0
        self._cf_log_packet_listeners = []
        self._mocap_frame_sequence = 0
        self._mocap_frame_listeners = []
        self.args = kwargs.get('controller_args', False)
        self.verbose = self.args.verbose

        self.group_kfs = {}

        log_dir = self.args.log_dir
        if not os.path.exists(log_dir):
            os.makedirs(log_dir, exist_ok=True)
        self.live_logger = LiveLogger(os.path.join(log_dir, f"{self.args.tag}.json"))
        self.get_git_version()

        # self.extra_markers = {}
        # for marker in self.args.extra_marker:
        #     self.extra_markers[marker['name']] = []

    def start(self, *args, **kwargs):
        self.live_logger.mark_start()

    def stop(self, *args, **kwargs):
        with self.cf_log_callback_lock:
            self._accepting_cf_log_callbacks = False
            self._cf_log_packet_listeners = []
            self._mocap_frame_listeners = []

        if self.cf_var_logger is not None:
            for log_config in self.cf_var_logger:
                log_config.stop()

        self.live_logger.close()

    def init_cf_logger(self, cf, cf_log_vars, cf_log_period=100):
        self.cf_log_data = copy.deepcopy(cf_log_vars)

        self.cf_var_logger = []
        for name, log_group in self.cf_log_data.items():
            # Match the ordinary logger's legacy-period compensation. cflib
            # divides period_in_ms by 10 before sending CONTROL_START_BLOCK,
            # while the deployed FLS firmware consumes that byte in 1 ms
            # units. Multiplying here preserves the configured period on that
            # firmware (for example 1 -> 10 -> byte 1 -> actual 1 ms).
            log_period = cf_log_period * 10
            if "log_period_ms" in log_group:
                log_period = log_group.pop("log_period_ms") * 10

            var_logger = LogConfig(name=f'{name}', period_in_ms=log_period)

            for par, conf in log_group.items():
                var_logger.add_variable(par, conf["type"])

            cf.log.add_config(var_logger)
            var_logger.data_received_cb.add_callback(self._cf_log_group_callback)
            var_logger.start()

            self.cf_var_logger.append(var_logger)

        logger.debug("logging activated")

    def add_log_group(self, name, *args, kf=False, **kwargs):
        self.groups[name] = []
        if kf:
            # controller.py renamed --fps to --tracker-camera-rate; the follow/test
            # controllers still pass --fps
            fps = getattr(self.args, 'fps', None) or getattr(self.args, 'tracker_camera_rate', 100)
            dt = 1 / fps
            self.group_kfs[name] = {
                'x': VelocityKalmanFilter(dt=dt, process_noise=1.0, measurement_noise=0.001 ** 2),
                'y': VelocityKalmanFilter(dt=dt, process_noise=1.0, measurement_noise=0.001 ** 2),
                'z': VelocityKalmanFilter(dt=dt, process_noise=1.0, measurement_noise=0.001 ** 2),
            }

    def add_log_entry(self, group_name, entry, *args, **kwargs):
        # Preserve the legacy/default-disabled timing path. The additional
        # shutdown serialization is needed only while an opt-in diagnostic
        # listener exists.
        listener_active = bool(
            getattr(self, '_cf_log_packet_listeners', ())
            or getattr(self, '_mocap_frame_listeners', ())
        )
        if listener_active:
            with self.cf_log_callback_lock:
                if not self._accepting_cf_log_callbacks:
                    return
                return self._add_log_entry(
                    group_name, entry, *args, **kwargs
                )
        return self._add_log_entry(group_name, entry, *args, **kwargs)

    def _add_log_entry(self, group_name, entry, *args, **kwargs):
        if (
                group_name == 'frames'
                and isinstance(entry, dict)
                and entry.get('tvec') is not None
        ):
            listeners = tuple(
                getattr(self, '_mocap_frame_listeners', ())
            )
            if listeners:
                received_at = time.time()
                received_monotonic = time.monotonic()
                sequence = getattr(self, '_mocap_frame_sequence', 0)
                self._mocap_frame_sequence = sequence + 1
                # Only the opt-in diagnostic path adds these fields. The
                # payload already passed through send_extpos/send_extpose.
                entry = copy.deepcopy(entry)
                entry['mocap_frame_sequence'] = sequence
                entry['host_receive_time_s'] = received_at
                entry['host_receive_monotonic_s'] = received_monotonic
                packet = MocapFramePacket(
                    sequence=sequence,
                    group=group_name,
                    host_receive_time_s=received_at,
                    data=_immutable_copy(entry),
                    cf_timestamp_ms=(
                        None
                        if entry.get('cf_timestamp_ms') is None
                        else int(entry['cf_timestamp_ms'])
                    ),
                    host_receive_monotonic_s=received_monotonic,
                )
                for listener in listeners:
                    try:
                        listener(packet)
                    except Exception:
                        logger.exception(
                            'read-only mocap frame listener failed; ignored'
                        )
        if group_name not in self.groups.keys():
            self.groups[group_name] = []
        kf = self.group_kfs.get(group_name)
        if (
            kf is not None and entry is not None
            and entry.get('tvec', None) is not None
        ):
            entry['vel'] = self._update_kf(entry['tvec'], kf)

        self.groups[group_name].append(entry)

        if self.live_logger:
            self.live_logger.write({
                "type": group_name,
                'name': kwargs.get('name', None),
                "data": entry,
            })

    def get_latest_group_log_data(self, log_group=None):
        if self.cf_log_data is None:
            return {}

        if log_group is None:
            log_group = list(self.cf_log_data.keys())[0]

        latest_values = {}
        for par, info in self.cf_log_data[log_group].items():
            data_list = info.get("data", [])
            if data_list:
                latest_values[par] = data_list[-1]
            else:
                latest_values[par] = None
        return latest_values

    def get_latest_group_log_time(self, log_group):
        """Return host receipt time of the latest Crazyflie log packet."""
        return self.cf_log_group_times.get(log_group)

    def get_nearest_group_log_data(self, log_group, timestamp):
        """Return the packet nearest a host-clock timestamp and its time skew."""
        packets = self.cf_log_group_packets.get(log_group)
        if not packets:
            return None, None
        with self.cf_log_packet_lock:
            snapshot = list(packets)
        if not snapshot:
            return None, None
        packet = min(snapshot, key=lambda candidate: abs(candidate['time'] - timestamp))
        return packet.copy(), abs(packet['time'] - timestamp)

    def get_latest_cf_log_data(self, group_name, param_name):
        if self.cf_log_data is None:
            return None
        group = self.cf_log_data.get(group_name)
        if group is None:
            # interaction config names this group POS_ACC, not VEL_POS
            group = next((g for g in self.cf_log_data.values() if param_name in g), None)
        if group is None or param_name not in group:
            return None
        data = group[param_name].get("data")
        return data[-1] if data else None

    def add_cf_packet_listener(self, listener: Callable[[CfLogPacket], None]):
        """Register a read-only listener and return an idempotent unsubscribe.

        Listeners run inside the callback/shutdown serialization lock and must
        only enqueue or copy data.  Exceptions are isolated from flight logging.
        """
        if not callable(listener):
            raise TypeError("listener must be callable")
        with self.cf_log_callback_lock:
            if not self._accepting_cf_log_callbacks:
                raise RuntimeError("Crazyflie logging is shutting down")
            self._cf_log_packet_listeners.append(listener)

        def unsubscribe():
            with self.cf_log_callback_lock:
                if listener in self._cf_log_packet_listeners:
                    self._cf_log_packet_listeners.remove(listener)

        return unsubscribe

    def add_mocap_frame_listener(
            self, listener: Callable[[MocapFramePacket], None]):
        """Register a diagnostic listener for main-drone Vicon frames."""
        if not callable(listener):
            raise TypeError('listener must be callable')
        with self.cf_log_callback_lock:
            if not self._accepting_cf_log_callbacks:
                raise RuntimeError('motion-capture logging is shutting down')
            self._mocap_frame_listeners.append(listener)

        def unsubscribe():
            with self.cf_log_callback_lock:
                if listener in self._mocap_frame_listeners:
                    self._mocap_frame_listeners.remove(listener)

        return unsubscribe

    def _cf_log_group_callback(self, timestamp, data, log_conf):
        with self.cf_log_callback_lock:
            if not self._accepting_cf_log_callbacks:
                return

            cur_time = time.time()
            group_name = log_conf.name
            transport_timestamp = int(timestamp)
            effective_timestamp = transport_timestamp
            source_timestamp = None
            source_transport_skew_ms = None
            source_timestamp_basis = None
            source_snapshot_atomic = False
            source_timestamp_error = None
            epoch_keys = _CONTACT_SOURCE_EPOCH_KEYS.get(group_name)
            epoch_key = None
            if epoch_keys is not None:
                epoch_key = next(
                    (key for key in epoch_keys if key in data), epoch_keys[0]
                )
                source_timestamp, source_transport_skew_ms = (
                    reconstruct_contact_source_timestamp_ms(
                        transport_timestamp, data.get(epoch_key)
                    )
                )
                if source_timestamp is None:
                    source_timestamp_error = (
                        'missing_invalid_or_ambiguous_firmware_source_epoch'
                    )
                else:
                    effective_timestamp = source_timestamp
                    source_timestamp_basis = CONTACT_SOURCE_TIMESTAMP_BASIS
                    source_snapshot_atomic = True
            self.cf_log_group_times[group_name] = cur_time
            data['time'] = cur_time
            with self.cf_log_packet_lock:
                self.cf_log_group_packets[group_name].append(data.copy())
            if group_name in self.cf_log_data.keys():
                # Append data to each variable in the group
                for var_name, var_info in self.cf_log_data[group_name].items():
                    if var_name in data:
                        var_info['data'].append(data[var_name])

            packet_sequence = None
            if getattr(self, '_cf_log_packet_listeners', None):
                listeners = tuple(self._cf_log_packet_listeners)
                # Preserve the exact default-disabled callback path: immutable
                # packet allocation and global sequencing exist only while an
                # opt-in shadow listener is registered.
                sequence = getattr(self, '_cf_log_packet_sequence', 0)
                self._cf_log_packet_sequence = sequence + 1
                packet_sequence = sequence
                cur_monotonic = time.monotonic()
                packet_data = MappingProxyType({
                    key: value for key, value in data.items() if key != 'time'
                })
                packet = CfLogPacket(
                    sequence=sequence,
                    group=group_name,
                    cf_timestamp_ms=effective_timestamp,
                    host_receive_time_s=cur_time,
                    data=packet_data,
                    host_receive_monotonic_s=cur_monotonic,
                    transport_cf_timestamp_ms=transport_timestamp,
                    source_cf_timestamp_basis=source_timestamp_basis,
                    source_snapshot_atomic=source_snapshot_atomic,
                )
                for listener in listeners:
                    try:
                        listener(packet)
                    except Exception:
                        logger.exception(
                            "read-only Crazyflie packet listener failed; ignored"
                        )

            if self.live_logger:
                # Preserve both clocks for offline delay/jitter analysis.  The
                # Crazyflie callback timestamp is the raw 24-bit millisecond
                # counter (it wraps); it is not a Unix time or a measured delay.
                # Keep these fields out of runtime packet buffers so state age,
                # nearest-packet selection, and control timing remain unchanged.
                saved_data = dict(data)
                saved_data['cf_timestamp_ms'] = effective_timestamp
                saved_data['transport_cf_timestamp_ms'] = transport_timestamp
                if epoch_key is not None:
                    saved_data['source_cf_timestamp_ms'] = source_timestamp
                    saved_data['source_cf_timestamp_basis'] = (
                        source_timestamp_basis
                    )
                    saved_data['source_snapshot_atomic'] = (
                        source_snapshot_atomic
                    )
                    saved_data['source_cf_transport_skew_ms'] = (
                        source_transport_skew_ms
                    )
                    if source_timestamp_error is not None:
                        saved_data['source_cf_timestamp_error'] = (
                            source_timestamp_error
                        )
                saved_data['host_receive_time_s'] = cur_time
                if packet_sequence is not None:
                    saved_data['cf_packet_sequence'] = packet_sequence
                    saved_data['host_receive_monotonic_s'] = cur_monotonic
                self.live_logger.write({
                    "type": 'state', "group": group_name, "data": saved_data,
                })

    def _update_kf(self, pos, kf):
        return [axis_kf.update(p) for p, axis_kf in zip(pos, kf.values())]

    import subprocess

    def get_git_version(self):
        try:
            self.add_log_group("git")
            output = subprocess.check_output(['git', 'rev-parse', 'HEAD']).decode('ascii').strip()

            if self.live_logger:
                self.live_logger.write({"type": 'git', "data": {'version': output}})
            return
        except (subprocess.CalledProcessError, FileNotFoundError):
            return "Git is not installed or not found in PATH."
