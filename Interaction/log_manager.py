import copy
from bisect import bisect_left
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
import math

logger = logging.getLogger(__name__)


CF_TIMESTAMP_MODULUS_MS = 1 << 24
CF_LOG_TRANSPORT_TIMESTAMP_BASIS = 'crazyflie_log_transport_tick_v1'
CONTACT_SOURCE_TIMESTAMP_BASIS = (
    'firmware_latched_stabilizer_tick_low16_v1'
)
CONTACT_SOURCE_MAX_TRANSPORT_SKEW_MS = 100
# The wider bound above is only sufficient to reconstruct a low-16 producer
# tick for diagnostics.  A packet used by the post-release authority path must
# also prove a causal, tightly bounded trip from producer latch to CRTP log
# transport.  Keep this as a non-configurable ceiling so a mission cannot
# silently relax it.
CONTACT_SOURCE_AUTHORITY_MAX_TRANSPORT_SKEW_MS = 5
CRAZYSIM_CF_TIMESTAMP_BASIS = 'crazysim_device_clock_v1'
VICON_CAPTURE_TO_CF_TIMESTAMP_BASIS = (
    'vicon_capture_to_cf_calibrated_v1'
)
TRUSTED_MOCAP_CF_TIMESTAMP_BASES = frozenset({
    CRAZYSIM_CF_TIMESTAMP_BASIS,
    VICON_CAPTURE_TO_CF_TIMESTAMP_BASIS,
})
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
    cf_timestamp_basis: str | None = None
    cf_timestamp_uncertainty_ms: float | None = None


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
        self.cf_log_group_packet_metadata = collections.defaultdict(
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
        curve_error = None
        if getattr(self, 'curve_recorder', None) is not None:
            try:
                self.curve_recorder.close()
            except Exception as error:
                curve_error = error
        with self.cf_log_callback_lock:
            self._accepting_cf_log_callbacks = False
            self._cf_log_packet_listeners = []
            self._mocap_frame_listeners = []

        if self.cf_var_logger is not None:
            for log_config in self.cf_var_logger:
                log_config.stop()

        self.live_logger.close()
        if curve_error is not None:
            raise curve_error

    def init_cf_logger(self, cf, cf_log_vars, cf_log_period=100):
        self.cf_log_data = copy.deepcopy(cf_log_vars)

        self.cf_var_logger = []
        for name, log_group in self.cf_log_data.items():
            # The deployed FLS firmware consumes the cflib period byte in 1 ms
            # units and therefore needs the legacy x10 compensation. CrazySim
            # SITL follows the upstream 10 ms unit, so applying that workaround
            # there would turn a requested 10 ms control stream into 100 ms.
            period_scale = (
                1
                if getattr(getattr(self, 'args', None), 'crazysim', False)
                else 10
            )
            log_period = cf_log_period * period_scale
            if "log_period_ms" in log_group:
                log_period = (
                    log_group.pop("log_period_ms") * period_scale
                )

            var_logger = LogConfig(name=f'{name}', period_in_ms=log_period)

            for par, conf in log_group.items():
                var_logger.add_variable(par, conf["type"])

            cf.log.add_config(var_logger)
            var_logger.data_received_cb.add_callback(self._cf_log_group_callback)
            var_logger.start()

            self.cf_var_logger.append(var_logger)

        logger.debug("logging activated")

    def add_log_group(self, name, *args, kf=False,
                      kf_use_mocap_elapsed_dt=False, **kwargs):
        self.groups[name] = []
        elapsed_groups = getattr(self, '_kf_mocap_elapsed_groups', None)
        if elapsed_groups is None:
            elapsed_groups = self._kf_mocap_elapsed_groups = set()
            self._kf_mocap_last_epoch_s = {}
        elapsed_groups.discard(name)
        self._kf_mocap_last_epoch_s.pop(name, None)
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
            if name == 'frames' and kf_use_mocap_elapsed_dt:
                elapsed_groups.add(name)

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
                    cf_timestamp_basis=entry.get('cf_timestamp_basis'),
                    cf_timestamp_uncertainty_ms=entry.get(
                        'cf_timestamp_uncertainty_ms'
                    ),
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
            if group_name in getattr(self, '_kf_mocap_elapsed_groups', ()):
                entry['vel'] = self._update_mocap_elapsed_kf(
                    group_name, entry, kf)
            else:
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

    def get_nearest_group_log_metadata(self, log_group, timestamp):
        """Return timing/provenance without changing runtime data packets."""
        metadata = getattr(self, 'cf_log_group_packet_metadata', None)
        if metadata is None:
            return None, None
        packets = metadata.get(log_group)
        if not packets:
            return None, None
        with self.cf_log_packet_lock:
            snapshot = list(packets)
        if not snapshot:
            return None, None
        packet = min(
            snapshot,
            key=lambda candidate: abs(candidate['time'] - timestamp),
        )
        return packet.copy(), abs(packet['time'] - timestamp)

    def get_nearest_group_log_data_by_cf_timestamp(
            self, log_group, cf_timestamp_ms):
        """Return the packet nearest a Crazyflie-clock epoch.

        Host callback order is not a synchronization clock: separate 100 Hz
        log blocks can be delayed independently by Python scheduling even
        though their firmware samples are adjacent.  Pair the immutable
        timing sidecar with its data packet and compare the wrapping 24-bit
        log timestamp instead.
        """
        metadata = getattr(self, 'cf_log_group_packet_metadata', None)
        data_packets = self.cf_log_group_packets.get(log_group)
        metadata_packets = None if metadata is None else metadata.get(log_group)
        if not data_packets or not metadata_packets:
            return None, None
        with self.cf_log_packet_lock:
            data_snapshot = list(data_packets)
            metadata_snapshot = list(metadata_packets)
        if not data_snapshot or len(data_snapshot) != len(metadata_snapshot):
            return None, None
        reference = int(cf_timestamp_ms) & 0xFFFFFF

        def signed_delta_ms(candidate):
            value = candidate.get('cf_timestamp_ms')
            if value is None:
                return None
            return ((int(value) - reference + 0x800000) & 0xFFFFFF) - 0x800000

        candidates = []
        for data, timing in zip(data_snapshot, metadata_snapshot):
            delta_ms = signed_delta_ms(timing)
            if delta_ms is not None:
                candidates.append((abs(delta_ms), data))
        if not candidates:
            return None, None
        skew_ms, packet = min(candidates, key=lambda item: item[0])
        return packet.copy(), 0.001 * float(skew_ms)

    def get_latest_paired_group_log_data(
            self, log_group, paired_group, *, max_skew_s):
        """Atomically select the newest complete device-clock packet pair.

        Independent callbacks may have published only half of the newest
        pair. Search retained history instead of combining that half with a
        different cycle. Preserve the reference packet's ORIGINAL host time:
        callers must still apply their existing age limits, even when newer
        unmatched packets keep arriving. This method does not refresh age.
        """
        if not math.isfinite(max_skew_s) or max_skew_s < 0:
            raise ValueError('invalid packet-pair skew limit')
        with self.cf_log_packet_lock:
            metadata = getattr(self, 'cf_log_group_packet_metadata', {})
            reference = list(self.cf_log_group_packets.get(log_group, ()))
            reference_meta = list(metadata.get(log_group, ()))
            paired = list(self.cf_log_group_packets.get(paired_group, ()))
            paired_meta = list(metadata.get(paired_group, ()))
        if (not reference or not paired
                or len(reference) != len(reference_meta)
                or len(paired) != len(paired_meta)):
            return None
        anchor = reference_meta[-1].get('cf_timestamp_ms')
        if anchor is None:
            return None

        def epoch_offset(timing):
            epoch = timing.get('cf_timestamp_ms')
            if epoch is None:
                return None
            return ((int(epoch) - int(anchor) + 0x800000) & 0xFFFFFF) - 0x800000

        # Sort once, then use binary search rather than scanning both history
        # buffers for every candidate. Epochs are unwrapped around the latest
        # reference so pairing also works across the CRTP 24-bit clock wrap.
        candidates = []
        for data, timing in zip(paired, paired_meta):
            offset = epoch_offset(timing)
            if offset is not None:
                candidates.append((offset, data))
        candidates.sort(key=lambda item: item[0])
        if not candidates:
            return None
        epochs = [item[0] for item in candidates]
        for data, timing in zip(reversed(reference), reversed(reference_meta)):
            epoch = epoch_offset(timing)
            if epoch is None:
                continue
            index = bisect_left(epochs, epoch)
            neighbors = candidates[max(0, index - 1):index + 1]
            nearest_epoch, nearest = min(neighbors, key=lambda item: abs(item[0] - epoch))
            skew_s = .001 * abs(nearest_epoch - epoch)
            if skew_s <= max_skew_s:
                return data.copy(), timing.copy(), nearest.copy(), skew_s
        return None

    def get_latest_cf_log_data(self, group_name, param_name):
        if self.cf_log_data is None:
            return None
        # Landing still asks for legacy stateEstimate XYZ. In the opt-in
        # firmware-auto-brake subscription those coordinates arrive as
        # stateEstimateZ millimetres instead of a third position log block.
        if (
            group_name == 'VEL_POS'
            and param_name in (
                'stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z'
            )
            and 'FIRMWARE_KIN' in self.cf_log_data
        ):
            axis = param_name.rsplit('.', 1)[1]
            values = self.cf_log_data['FIRMWARE_KIN'][
                f'stateEstimateZ.{axis}']['data']
            return 0.001 * values[-1] if values else None
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
                if not hasattr(self, 'cf_log_group_packet_metadata'):
                    # Some focused/offline fixtures intentionally construct a
                    # minimal logger without running __init__. Keep the new
                    # provenance sidecar backward compatible with that path.
                    self.cf_log_group_packet_metadata = (
                        collections.defaultdict(
                            lambda: collections.deque(maxlen=1000)
                        )
                    )
                self.cf_log_group_packets[group_name].append(data.copy())
                self.cf_log_group_packet_metadata[group_name].append({
                    'time': cur_time,
                    'cf_timestamp_ms': effective_timestamp,
                    'transport_cf_timestamp_ms': transport_timestamp,
                    'cf_timestamp_basis': (
                        source_timestamp_basis
                        or CF_LOG_TRANSPORT_TIMESTAMP_BASIS
                    ),
                    'source_snapshot_atomic': source_snapshot_atomic,
                })
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

    def _update_mocap_elapsed_kf(self, group_name, entry, kf):
        """Match firmware mirror epochs without changing frame timestamps.

        This is an opt-in host diagnostic, not firmware command authority.
        Preserve filter history across gaps; reject missing/nonmonotonic epochs
        rather than inventing a dt from logger scheduling or wall-clock time.
        """
        epoch = (entry.get('mocap_timing') or {}).get(
            'wait_return_monotonic_s')
        previous = self._kf_mocap_last_epoch_s.get(group_name)
        status = None
        if (isinstance(epoch, bool) or not isinstance(epoch, (int, float))
                or not math.isfinite(epoch)):
            status = 'missing_or_invalid_epoch'
        elif previous is not None and epoch <= previous:
            status = 'nonmonotonic_epoch'
        if status is not None:
            entry['velocity_kf_timing'] = {
                'basis': 'pi_mocap_wait_return_monotonic',
                'update_applied': False, 'status': status, 'dt_s': None,
            }
            return [float(axis.x[1, 0]) for axis in kf.values()]
        dt = None if previous is None else epoch - previous
        velocity = [axis.update(position, dt=dt)
                    for position, axis in zip(entry['tvec'], kf.values())]
        self._kf_mocap_last_epoch_s[group_name] = float(epoch)
        entry['velocity_kf_timing'] = {
            'basis': 'pi_mocap_wait_return_monotonic',
            'update_applied': True,
            'status': 'initial_nominal_step' if dt is None else 'elapsed_step',
            'dt_s': next(iter(kf.values())).dt,
        }
        return velocity

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
