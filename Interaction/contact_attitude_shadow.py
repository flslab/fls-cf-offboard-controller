"""Bounded, read-only runtime adapter for contact attitude shadow estimation."""

from __future__ import annotations

import collections
import copy
from dataclasses import asdict, dataclass
import functools
import math
import numbers
import threading
import time
from typing import Callable, Sequence

import numpy as np

from Interaction.contact_attitude_observer import (
    CF_TIMESTAMP_MODULUS_MS,
    ContactAttitudeConfig,
    ContactAttitudeObserver,
    legacy_rpy_from_quaternion,
    quaternion_from_native_rpy,
)
from Interaction.contact_attitude_experiment import (
    INERTIAL_POSITION,
    ONBOARD_MIRROR,
    experiment_run_config,
)
from Interaction.log_manager import (
    CONTACT_SOURCE_TIMESTAMP_BASIS,
    CfLogPacket,
    MocapFramePacket,
)
from Interaction.post_release_inertial_ekf import (
    PostReleaseEkfConfig,
    PostReleaseInertialEkf,
)


@dataclass(frozen=True)
class ContactAttitudeShadowConfig:
    queue_capacity: int = 4096
    max_drain_packets: int = 256
    max_drain_time_s: float = 0.004
    max_release_processing_time_s: float = 0.010
    join_tolerance_ms: float = 15.0
    state_join_tolerance_ms: float = 15.0
    history_capacity: int = 512
    gyro_reorder_wait_s: float = 0.03
    release_attitude_continuity_max_deg: float = 0.01
    max_release_event_to_gyro_skew_s: float = 0.02
    max_release_replay_samples: int = 32
    max_position_replay_samples: int = 32
    max_packet_host_age_s: float = 0.25
    max_packet_host_skew_s: float = 0.05
    alignment_stationary_window_ms: float = 100.0
    alignment_min_state_samples: int = 5
    alignment_max_state_gap_ms: float = 25.0
    alignment_max_speed_m_s: float = 0.08
    alignment_max_position_span_m: float = 0.025
    alignment_max_nominal_yaw_error_deg: float = 5.0
    mode: str = INERTIAL_POSITION
    experiment_run: int | None = None
    vicon_orientation_forwarded: bool | None = None
    alignment_legacy_yaw_deg: float | None = None

    def __post_init__(self):
        finite_limits = (
            self.max_drain_time_s,
            self.max_release_processing_time_s,
            self.join_tolerance_ms,
            self.state_join_tolerance_ms,
            self.gyro_reorder_wait_s,
            self.release_attitude_continuity_max_deg,
            self.max_release_event_to_gyro_skew_s,
            self.max_packet_host_age_s,
            self.max_packet_host_skew_s,
            self.alignment_stationary_window_ms,
            self.alignment_max_state_gap_ms,
            self.alignment_max_speed_m_s,
            self.alignment_max_position_span_m,
            self.alignment_max_nominal_yaw_error_deg,
        )
        if any(
                isinstance(value, bool)
                or not isinstance(value, numbers.Real)
                or not math.isfinite(float(value))
                for value in finite_limits):
            raise ValueError("shadow timing and alignment limits must be finite")
        integer_limits = (
            self.queue_capacity,
            self.max_drain_packets,
            self.history_capacity,
            self.alignment_min_state_samples,
            self.max_release_replay_samples,
            self.max_position_replay_samples,
        )
        if any(
                isinstance(value, bool)
                or not isinstance(value, numbers.Integral)
                for value in integer_limits):
            raise ValueError("shadow packet and sample limits must be integers")
        if (
            self.queue_capacity < 32
            or self.history_capacity < 32
            or self.max_drain_packets < 1
        ):
            raise ValueError("queue and history capacity must be at least 32")
        if (
            self.join_tolerance_ms <= 0.0
            or self.state_join_tolerance_ms <= 0.0
            or self.max_drain_time_s <= 0.0
            or self.max_release_processing_time_s <= 0.0
            or self.gyro_reorder_wait_s <= 0.0
            or self.release_attitude_continuity_max_deg <= 0.0
            or self.max_release_event_to_gyro_skew_s <= 0.0
            or self.max_packet_host_age_s <= 0.0
            or self.max_packet_host_skew_s <= 0.0
            or self.alignment_stationary_window_ms <= 0.0
            or self.alignment_max_state_gap_ms <= 0.0
            or self.alignment_max_speed_m_s <= 0.0
            or self.alignment_max_position_span_m <= 0.0
            or self.alignment_max_nominal_yaw_error_deg <= 0.0
        ):
            raise ValueError("timestamp join tolerances must be positive")
        if self.alignment_min_state_samples < 2:
            raise ValueError("alignment needs at least two state samples")
        if (
            self.max_release_replay_samples < 1
            or self.max_position_replay_samples < 1
        ):
            raise ValueError("shadow replay sample limits must be positive")
        if self.mode not in (ONBOARD_MIRROR, INERTIAL_POSITION):
            raise ValueError(
                "shadow mode must be onboard_mirror or inertial_position"
            )
        if self.experiment_run is not None:
            protocol = experiment_run_config(self.experiment_run)
            if protocol['shadow_mode'] != self.mode:
                raise ValueError("shadow mode does not match experiment run")
            if (
                self.vicon_orientation_forwarded is not None
                and bool(self.vicon_orientation_forwarded)
                != protocol['vicon_orientation_forwarded']
            ):
                raise ValueError(
                    "Vicon orientation routing does not match experiment run"
                )
        if (
            self.alignment_legacy_yaw_deg is not None
            and not math.isfinite(float(self.alignment_legacy_yaw_deg))
        ):
            raise ValueError('alignment yaw must be finite')


def _contain_shadow_errors(operation):
    """Make every main-loop-facing shadow operation fail observationally."""
    def decorate(function):
        @functools.wraps(function)
        def contained(self, *args, **kwargs):
            try:
                return function(self, *args, **kwargs)
            except Exception as error:
                self._failure_count += 1
                reason = (
                    f"{operation}_failed:{type(error).__name__}"
                )
                self._fatal_reason = self._fatal_reason or reason
                self._invalid_reason = reason
                return self.snapshot()
        return contained
    return decorate


class ContactAttitudeShadow:
    """Consumes immutable log packets and produces diagnostics only.

    The class exposes no commander and returns no control setpoint.  A cflib
    callback calls :meth:`enqueue_packet`; the ordinary interaction loop calls
    :meth:`drain` so expensive math never blocks packet receipt.
    """

    def __init__(
            self, config: ContactAttitudeShadowConfig | None = None,
            observer_config: ContactAttitudeConfig | None = None,
            ekf_config: PostReleaseEkfConfig | None = None,
            report: Callable[[dict], None] | None = None,
            clock: Callable[[], float] = time.monotonic,
            perf_clock: Callable[[], float] = time.perf_counter,
    ):
        self.config = config or ContactAttitudeShadowConfig()
        self.observer = ContactAttitudeObserver(observer_config)
        self.ekf_config = ekf_config or PostReleaseEkfConfig()
        self.report = report
        self._clock = clock
        self._perf_clock = perf_clock
        self._queue = collections.deque(maxlen=self.config.queue_capacity)
        self._queue_lock = threading.Lock()
        self._latest_accel = None
        self._accel_packets = collections.deque(maxlen=64)
        self._latest_gyro = None
        self._pending_gyros = collections.deque(
            maxlen=self.config.history_capacity
        )
        self._latest_position = None
        self._position_packets = collections.deque(maxlen=64)
        self._latest_velocity = None
        self._velocity_packets = collections.deque(maxlen=64)
        self._yaw_packets = collections.deque(maxlen=64)
        self._state_seed_packets = collections.deque(maxlen=64)
        self._latest_mocap = None
        self._mocap_frames = collections.deque(maxlen=64)
        self._pending_positions = collections.deque(maxlen=64)
        self._contact_imu_history = collections.deque(
            maxlen=self.config.history_capacity
        )
        self._ekf_history = collections.deque(
            maxlen=self.config.history_capacity
        )
        self._last_ekf_host_receive_monotonic_s = None
        self._latest_onboard_state = None
        self._onboard_state_history = collections.deque(
            maxlen=self.config.history_capacity
        )
        self._onboard_state_sequence = 0
        self._mirror_phase = 'tracking'
        self._mirror_previous_phase = None
        self._ekf = None
        self._release_snapshot = None
        self._pending_release = None
        self._contact_candidate_origin = None
        self._pre_candidate_observer = None
        self._dropped_packets = 0
        self._skipped_position_packets = 0
        self._last_position_measurement_skew_ms = None
        self._last_position_timing_basis = None
        self._strict_position_time_update_count = 0
        self._approximate_position_time_update_count = 0
        self._last_strict_position_update_cf_timestamp_ms = None
        self._last_processed_position_cf_timestamp_ms = None
        self._last_processed_approximate_position_time_s = None
        self._failure_count = 0
        self._drain_budget_exceeded_count = 0
        self._release_budget_exceeded_count = 0
        self._last_drain_duration_s = 0.0
        self._max_drain_duration_s = 0.0
        self._fatal_reason = None
        self._invalid_reason = None
        self._alignment_gate_metrics = None

    @staticmethod
    def _timestamp_distance_ms(left: int, right: int) -> int:
        return abs(ContactAttitudeShadow._timestamp_offset_ms(left, right))

    @staticmethod
    def _timestamp_offset_ms(left: int, right: int) -> int:
        """Return signed modular ``left - right`` in milliseconds."""
        delta = (int(left) - int(right)) % CF_TIMESTAMP_MODULUS_MS
        if delta >= CF_TIMESTAMP_MODULUS_MS // 2:
            delta -= CF_TIMESTAMP_MODULUS_MS
        return delta

    @staticmethod
    def _packet_monotonic_time(packet):
        value = getattr(packet, 'host_receive_monotonic_s', None)
        if value is None:
            # Backward-compatible offline fixtures carry only one host clock.
            value = packet.host_receive_time_s
        value = float(value)
        if not math.isfinite(value):
            raise ValueError('packet host monotonic time must be finite')
        return value

    def enqueue_packet(self, packet: CfLogPacket) -> None:
        if self.config.mode == ONBOARD_MIRROR:
            return
        if packet.group not in (
            'GYRO_1KHZ', 'ACC_ALIGN', 'POS_ACC', 'VEL_ORI',
            'CONTACT_STATE_SEED',
        ):
            return
        if (
            self.config.experiment_run in (2, 3)
            and packet.group in (
                'GYRO_1KHZ', 'ACC_ALIGN', 'CONTACT_STATE_SEED',
            )
            and not self._packet_has_trusted_source_timestamp(packet)
        ):
            self._fatal_reason = self._fatal_reason or (
                'firmware_source_timestamp_provenance_missing'
            )
            self._invalid_reason = (
                'firmware_source_timestamp_provenance_missing'
            )
            return
        with self._queue_lock:
            if len(self._queue) == self._queue.maxlen:
                self._dropped_packets += 1
                self._fatal_reason = (
                    self._fatal_reason or "shadow_queue_overflow"
                )
                self._invalid_reason = "shadow_queue_overflow"
            self._queue.append(packet)

    def enqueue_mocap_frame(self, packet: MocapFramePacket) -> None:
        """Queue the exact Vicon frame already forwarded by the controller."""
        if packet.group != 'frames':
            return
        with self._queue_lock:
            if len(self._queue) == self._queue.maxlen:
                self._dropped_packets += 1
                self._fatal_reason = (
                    self._fatal_reason or "shadow_queue_overflow"
                )
                self._invalid_reason = "shadow_queue_overflow"
            self._queue.append(packet)

    @_contain_shadow_errors("drain")
    def drain(self) -> dict:
        started = self._perf_clock()
        deadline = started + self.config.max_drain_time_s
        with self._queue_lock:
            packets = list(self._queue)
            self._queue.clear()
        if len(packets) > self.config.max_drain_packets:
            self._dropped_packets += len(packets)
            self._drain_budget_exceeded_count += 1
            self._fatal_reason = self._fatal_reason or (
                'shadow_drain_packet_budget_exceeded'
            )
            self._invalid_reason = 'shadow_drain_packet_budget_exceeded'
            self._pending_gyros.clear()
            self._pending_positions.clear()
            return self._finish_drain(started)
        # Log blocks are independent CRTP streams, so callback arrival order is
        # not a device-time total order.  Ingest the lower-rate side channels
        # first, then consume gyro packets in their own stream order.  A gyro
        # that still lacks a causal accelerometer sample is retained briefly;
        # it is never paired with a future accelerometer sample.
        for packet in packets:
            if self._perf_clock() > deadline:
                return self._abort_drain_budget(started, packets)
            if (
                packet.group == 'GYRO_1KHZ'
                and self._is_packed_contact_imu(packet)
            ):
                # The live 1 kHz format carries gyro, accelerometer, and the
                # complete onboard release seed in one atomic CRTP packet.
                # Publish its side channels before the gyro pass below so the
                # exact same epoch is selected without cross-stream joining.
                self._position(packet)
                self._velocity(packet)
                self._latest_accel = packet
                self._accel_packets.append(packet)
                self._state_seed_packets.append(packet)
            elif packet.group == 'frames':
                self._process_mocap_frame(packet)
            elif packet.group == 'ACC_ALIGN':
                self._latest_accel = packet
                self._accel_packets.append(packet)
            elif packet.group == 'POS_ACC':
                self._process_position(packet)
            elif packet.group == 'VEL_ORI':
                self._process_velocity(packet)
            elif packet.group == 'CONTACT_STATE_SEED':
                # Validate all six fields together; this packet is the only
                # strict release seed used by the enabled runtime path.
                self._position(packet)
                self._velocity(packet)
                self._state_seed_packets.append(packet)
        for packet in packets:
            if self._perf_clock() > deadline:
                return self._abort_drain_budget(started, packets)
            if packet.group != 'GYRO_1KHZ':
                continue
            if len(self._pending_gyros) == self._pending_gyros.maxlen:
                self._dropped_packets += 1
                self._fatal_reason = self._fatal_reason or (
                    'shadow_gyro_reorder_overflow'
                )
                self._invalid_reason = 'shadow_gyro_reorder_overflow'
            self._pending_gyros.append(packet)
        if not self._flush_pending_gyros(deadline):
            return self._abort_drain_budget(started, packets)
        if not self._fuse_pending_positions(deadline):
            return self._abort_drain_budget(started, packets)
        result = self._finish_drain(started)
        if self.report is not None:
            try:
                self.report(result)
            except Exception:
                # Shadow diagnostics must never interrupt the existing flight
                # loop, even when the diagnostic log sink is closing/failing.
                self._fatal_reason = (
                    self._fatal_reason or "shadow_report_failed"
                )
                self._invalid_reason = "shadow_report_failed"
                result = self.snapshot()
        return result

    def _finish_drain(self, started):
        duration = max(0.0, float(self._perf_clock() - started))
        self._last_drain_duration_s = duration
        self._max_drain_duration_s = max(
            self._max_drain_duration_s, duration
        )
        return self.snapshot()

    def _abort_drain_budget(self, started, packets):
        self._drain_budget_exceeded_count += 1
        self._fatal_reason = self._fatal_reason or (
            'shadow_drain_time_budget_exceeded'
        )
        self._invalid_reason = 'shadow_drain_time_budget_exceeded'
        self._dropped_packets += len(self._pending_gyros)
        self._pending_gyros.clear()
        self._pending_positions.clear()
        return self._finish_drain(started)

    def _flush_pending_gyros(self, deadline=None):
        while self._pending_gyros:
            if deadline is not None and self._perf_clock() > deadline:
                return False
            packet = self._pending_gyros[0]
            joined = self._joined_accel(packet)
            observer_phase = self.observer.phase
            needs_accel = (
                self._ekf is not None
                or observer_phase in (
                    self.observer.ALIGNING,
                    self.observer.READY,
                    self.observer.CONTACT,
                )
            )
            if needs_accel and joined is None:
                age_s = float(self._clock()) - float(
                    self._packet_monotonic_time(packet)
                )
                if age_s < -self.config.max_packet_host_skew_s:
                    # The injected/test clock has not reached this callback.
                    break
                if age_s <= self.config.gyro_reorder_wait_s:
                    break
                self._pending_gyros.popleft()
                if self._ekf is not None:
                    self._fatal_reason = self._fatal_reason or (
                        'post_release_accel_join_missing'
                    )
                    self._invalid_reason = 'post_release_accel_join_missing'
                elif observer_phase == self.observer.CONTACT:
                    # Contact attitude itself remains gyro-only, but release
                    # initialization may need to replay this interval.  Losing
                    # its measured specific force would make that replay a
                    # different dynamical system, so fail the shadow instead of
                    # silently substituting commands or zero acceleration.
                    self._fatal_reason = self._fatal_reason or (
                        'contact_accel_join_missing'
                    )
                    self._invalid_reason = 'contact_accel_join_missing'
                # Missing alignment accelerometer data merely leaves alignment
                # unready. It is not a flight/controller failure.
                continue
            self._pending_gyros.popleft()
            self._process_gyro(packet, joined)
        return True

    def _joined_accel(self, gyro_packet):
        candidates = []
        for packet in self._accel_packets:
            age_ms = self._timestamp_offset_ms(
                gyro_packet.cf_timestamp_ms, packet.cf_timestamp_ms
            )
            if 0 <= age_ms <= self.config.join_tolerance_ms:
                candidates.append((age_ms, packet))
        if not candidates:
            return None
        _age_ms, packet = min(candidates, key=lambda item: item[0])
        try:
            return (
                [
                    self._packet_value(
                        packet, 'contactImu.ax', 'contactAccel.x', 'acc.x'
                    ),
                    self._packet_value(
                        packet, 'contactImu.ay', 'contactAccel.y', 'acc.y'
                    ),
                    self._packet_value(
                        packet, 'contactImu.az', 'contactAccel.z', 'acc.z'
                    ),
                ],
                (
                    self._packet_yaw(packet, gyro_packet.cf_timestamp_ms)
                    if self.config.alignment_legacy_yaw_deg is None
                    else self.config.alignment_legacy_yaw_deg
                ),
            )
        except KeyError:
            self._invalid_reason = "accel_packet_missing_field"
            return None

    @staticmethod
    def _packet_value(packet, *names):
        for name in names:
            if name in packet.data:
                return packet.data[name]
        raise KeyError(names[0])

    @staticmethod
    def _is_packed_contact_imu(packet):
        return all(
            name in packet.data for name in (
                'contactImu.gx', 'contactImu.gy', 'contactImu.gz',
                'contactImu.ax', 'contactImu.ay', 'contactImu.az',
                'contactImu.px', 'contactImu.py', 'contactImu.pz',
                'contactImu.vx', 'contactImu.vy', 'contactImu.vz',
                'contactImu.epoch',
            )
        )

    def _packet_yaw(self, packet, reference_timestamp):
        try:
            return self._packet_value(
                packet, 'contactAccel.yaw', 'stateEstimate.yaw'
            )
        except KeyError:
            candidates = []
            for yaw_packet in self._yaw_packets:
                age_ms = self._timestamp_offset_ms(
                    reference_timestamp, yaw_packet.cf_timestamp_ms
                )
                if 0 <= age_ms <= self.config.join_tolerance_ms:
                    candidates.append((age_ms, yaw_packet))
            if not candidates:
                raise
            _age_ms, yaw_packet = min(candidates, key=lambda item: item[0])
            return self._packet_value(yaw_packet, 'stateEstimate.yaw')

    @staticmethod
    def _packet_has_trusted_source_timestamp(packet):
        return (
            getattr(packet, 'source_snapshot_atomic', False) is True
            and getattr(packet, 'source_cf_timestamp_basis', None)
            == CONTACT_SOURCE_TIMESTAMP_BASIS
            and getattr(packet, 'transport_cf_timestamp_ms', None)
            is not None
        )

    @classmethod
    def _gyro(cls, packet):
        return [
            cls._packet_value(
                packet, 'contactImu.gx', 'contactGyro.x', 'gyro.x'
            ),
            cls._packet_value(
                packet, 'contactImu.gy', 'contactGyro.y', 'gyro.y'
            ),
            cls._packet_value(
                packet, 'contactImu.gz', 'contactGyro.z', 'gyro.z'
            ),
        ]

    @classmethod
    def _position(cls, packet):
        return [
            cls._packet_value(
                packet, 'contactImu.px', 'contactSeed.x', 'stateEstimate.x'
            ),
            cls._packet_value(
                packet, 'contactImu.py', 'contactSeed.y', 'stateEstimate.y'
            ),
            cls._packet_value(
                packet, 'contactImu.pz', 'contactSeed.z', 'stateEstimate.z'
            ),
        ]

    @classmethod
    def _velocity(cls, packet):
        return [
            cls._packet_value(
                packet, 'contactImu.vx', 'contactSeed.vx', 'stateEstimate.vx'
            ),
            cls._packet_value(
                packet, 'contactImu.vy', 'contactSeed.vy', 'stateEstimate.vy'
            ),
            cls._packet_value(
                packet, 'contactImu.vz', 'contactSeed.vz', 'stateEstimate.vz'
            ),
        ]

    def _process_position(self, packet):
        # Retain onboard position for comparison only. The custom EKF consumes
        # the raw Vicon tvec that was sent through extpos/extpose, never a
        # position already filtered by the onboard EKF.
        self._position(packet)
        self._latest_position = packet
        self._position_packets.append(packet)

    def _process_velocity(self, packet):
        self._velocity(packet)
        self._latest_velocity = packet
        self._velocity_packets.append(packet)
        if 'stateEstimate.yaw' in packet.data:
            yaw = float(packet.data['stateEstimate.yaw'])
            if math.isfinite(yaw):
                self._yaw_packets.append(packet)

    def _initial_alignment_state_gate(self, observer_estimate):
        """Require a stationary onboard-state window before first contact.

        An accelerometer alone cannot distinguish gravity from a constant
        translational acceleration.  The atomic position/velocity log block is
        therefore used only as a pre-contact stationarity certificate; it is
        never replayed as a command-derived velocity model.
        """
        reference = observer_estimate.cf_timestamp_ms
        metrics = {
            'reference_cf_timestamp_ms': reference,
            'passed': False,
            'reason': None,
        }
        self._alignment_gate_metrics = metrics

        def fail(reason):
            metrics['reason'] = reason
            return False, reason

        if reference is None:
            return fail('alignment_state_reference_missing')
        causal_samples = []
        for packet in self._state_seed_packets:
            age_ms = self._timestamp_offset_ms(
                reference, packet.cf_timestamp_ms
            )
            if age_ms >= 0:
                causal_samples.append((float(age_ms), packet))
        if not causal_samples:
            return fail('alignment_stationary_state_missing')
        causal_samples.sort(key=lambda item: item[0])
        newest_age_ms = causal_samples[0][0]
        samples = [
            item for item in causal_samples
            if item[0] - newest_age_ms <= (
                self.config.alignment_stationary_window_ms
            )
        ]
        covered_ms = samples[-1][0] - newest_age_ms
        ages_ms = [item[0] for item in samples]
        metrics.update({
            'state_sample_count': len(samples),
            'state_window_covered_ms': float(covered_ms),
            'newest_state_age_ms': float(newest_age_ms),
        })
        if len(set(ages_ms)) != len(ages_ms):
            return fail('alignment_state_timestamp_duplicate')
        state_gaps_ms = [
            later - earlier
            for earlier, later in zip(ages_ms, ages_ms[1:])
        ]
        max_state_gap_ms = max(state_gaps_ms, default=0.0)
        metrics['max_state_gap_ms'] = float(max_state_gap_ms)
        if (
            newest_age_ms > self.config.state_join_tolerance_ms
            or len(samples) < self.config.alignment_min_state_samples
            or covered_ms < self.config.alignment_stationary_window_ms
        ):
            return fail('alignment_stationary_window_incomplete')
        if max_state_gap_ms > self.config.alignment_max_state_gap_ms:
            return fail('alignment_state_sample_gap')
        now = float(self._clock())
        newest_host_age_s = now - self._packet_monotonic_time(samples[0][1])
        if (
            newest_host_age_s < -self.config.max_packet_host_skew_s
            or newest_host_age_s > self.config.max_packet_host_age_s
        ):
            return fail('alignment_stationary_state_host_stale')
        positions = np.asarray([
            self._position(packet) for _age, packet in samples
        ], dtype=float)
        velocities = np.asarray([
            self._velocity(packet) for _age, packet in samples
        ], dtype=float)
        if float(np.max(np.linalg.norm(velocities, axis=1))) > (
                self.config.alignment_max_speed_m_s):
            return fail('alignment_stationary_speed_exceeded')
        center = np.mean(positions, axis=0)
        if float(np.max(np.linalg.norm(positions - center, axis=1))) > (
                self.config.alignment_max_position_span_m):
            return fail('alignment_stationary_position_span_exceeded')
        metrics['max_speed_m_s'] = float(np.max(np.linalg.norm(
            velocities, axis=1
        )))
        metrics['max_position_radius_m'] = float(np.max(np.linalg.norm(
            positions - center, axis=1
        )))

        yaw_samples = []
        # Packed contactImu packets spend the full CRTP payload on gyro,
        # acceleration, position, velocity, and epoch. Read the independent
        # stationary-yaw gate from the existing lower-rate onboard VEL_ORI
        # stream; legacy ACC_ALIGN packets remain accepted for old replays.
        for packet in tuple(self._yaw_packets) + tuple(self._accel_packets):
            age_ms = self._timestamp_offset_ms(
                reference, packet.cf_timestamp_ms
            )
            if 0 <= age_ms - newest_age_ms <= (
                    self.config.alignment_stationary_window_ms):
                try:
                    yaw = float(self._packet_value(
                        packet, 'contactAccel.yaw', 'stateEstimate.yaw'
                    ))
                except KeyError:
                    continue
                if not math.isfinite(yaw):
                    return fail('alignment_onboard_yaw_invalid')
                yaw_samples.append((float(age_ms), yaw))
        if len(yaw_samples) < self.config.alignment_min_state_samples:
            return fail('alignment_onboard_yaw_window_incomplete')
        yaw_radians = np.radians([sample[1] for sample in yaw_samples])
        mean_sin = float(np.mean(np.sin(yaw_radians)))
        mean_cos = float(np.mean(np.cos(yaw_radians)))
        resultant = math.hypot(mean_sin, mean_cos)
        if resultant < self.observer.config.alignment_min_yaw_resultant:
            return fail('alignment_onboard_yaw_ambiguous')
        mean_yaw_deg = math.degrees(math.atan2(mean_sin, mean_cos))
        yaw_deviations = [
            self._wrapped_angle_error_deg(yaw, mean_yaw_deg)
            for _age, yaw in yaw_samples
        ]
        max_yaw_deviation = max(abs(value) for value in yaw_deviations)
        metrics.update({
            'onboard_yaw_sample_count': len(yaw_samples),
            'onboard_yaw_resultant': float(resultant),
            'onboard_mean_yaw_deg': float(mean_yaw_deg),
            'onboard_max_yaw_deviation_deg': float(max_yaw_deviation),
        })
        if max_yaw_deviation > (
                self.observer.config.alignment_max_yaw_deviation_deg):
            return fail('alignment_onboard_yaw_unstable')
        if self.config.alignment_legacy_yaw_deg is not None:
            nominal_error = abs(self._wrapped_angle_error_deg(
                mean_yaw_deg, self.config.alignment_legacy_yaw_deg
            ))
            metrics['nominal_yaw_error_deg'] = float(nominal_error)
            if nominal_error > (
                    self.config.alignment_max_nominal_yaw_error_deg):
                return fail('alignment_nominal_yaw_mismatch')
        metrics['passed'] = True
        return True, None

    @_contain_shadow_errors("update_onboard_state")
    def update_onboard_state(
            self, position_m, velocity_m_s, legacy_rpy_rad,
            angular_velocity_rad_s=None, state_time_s=None,
            host_receive_monotonic_s=None,
    ) -> dict:
        """Copy the synchronized onboard estimate for comparison/mirror mode."""
        position = np.asarray(position_m, dtype=float)
        velocity = np.asarray(velocity_m_s, dtype=float)
        rpy = np.asarray(legacy_rpy_rad, dtype=float)
        if any(
            value.shape != (3,) or not np.all(np.isfinite(value))
            for value in (position, velocity, rpy)
        ):
            raise ValueError('onboard state vectors must contain three finite values')
        angular_velocity = None
        if angular_velocity_rad_s is not None:
            angular_velocity = np.asarray(angular_velocity_rad_s, dtype=float)
            if (
                angular_velocity.shape != (3,)
                or not np.all(np.isfinite(angular_velocity))
            ):
                raise ValueError(
                    'onboard angular velocity must contain three finite values'
                )
        state_time = float(self._clock() if state_time_s is None else state_time_s)
        if not math.isfinite(state_time):
            raise ValueError('onboard state time must be finite')
        monotonic_time = float(
            self._clock()
            if host_receive_monotonic_s is None
            else host_receive_monotonic_s
        )
        if not math.isfinite(monotonic_time):
            raise ValueError('onboard host monotonic time must be finite')
        quaternion = quaternion_from_native_rpy(
            float(rpy[0]), -float(rpy[1]), float(rpy[2])
        )
        self._latest_onboard_state = {
            'source': 'synchronized_onboard_ekf_telemetry',
            'sequence': self._onboard_state_sequence,
            'state_time_s': state_time,
            'host_receive_monotonic_s': monotonic_time,
            'position_m': position.tolist(),
            'velocity_m_s': velocity.tolist(),
            'legacy_rpy_rad': rpy.tolist(),
            'legacy_rpy_deg': np.degrees(rpy).tolist(),
            'quaternion_wxyz': quaternion.tolist(),
            'quaternion_reconstructed_from_logged_euler': True,
            'angular_velocity_rad_s': (
                None if angular_velocity is None else angular_velocity.tolist()
            ),
        }
        self._onboard_state_history.append(copy.deepcopy(
            self._latest_onboard_state
        ))
        self._onboard_state_sequence += 1
        return self.snapshot()

    def _onboard_state_freshness(self):
        if self._latest_onboard_state is None:
            return False, 'onboard_mirror_state_missing', None
        age_s = float(self._clock()) - float(
            self._latest_onboard_state['host_receive_monotonic_s']
        )
        if not math.isfinite(age_s):
            return False, 'onboard_mirror_state_clock_invalid', age_s
        if age_s < -self.config.max_packet_host_skew_s:
            return False, 'onboard_mirror_state_from_future', age_s
        if age_s > self.config.max_packet_host_age_s:
            return False, 'onboard_mirror_state_stale', age_s
        return True, None, age_s

    def _processed_gyro_freshness(self, expected_timestamp=None):
        if self._pending_gyros:
            return False, 'contact_gyro_reorder_pending', None
        if self._latest_gyro is None:
            return False, 'contact_gyro_packet_missing', None
        if (
            expected_timestamp is not None
            and int(self._latest_gyro.cf_timestamp_ms)
            != int(expected_timestamp)
        ):
            return False, 'contact_gyro_epoch_mismatch', None
        age_s = float(self._clock()) - float(
            self._packet_monotonic_time(self._latest_gyro)
        )
        if not math.isfinite(age_s):
            return False, 'contact_gyro_host_clock_invalid', age_s
        if age_s < -self.config.max_packet_host_skew_s:
            return False, 'contact_gyro_packet_from_future', age_s
        if age_s > self.config.max_packet_host_age_s:
            return False, 'contact_gyro_packet_stale', age_s
        return True, None, age_s

    @staticmethod
    def _mocap_position(packet):
        position = np.asarray(packet.data['tvec'], dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError('mocap tvec must contain three finite values')
        return position.tolist()

    def _process_mocap_frame(self, packet):
        self._mocap_position(packet)
        quaternion = packet.data.get('quat')
        if quaternion is not None:
            quaternion = np.asarray(quaternion, dtype=float)
            if (
                quaternion.shape != (4,)
                or not np.all(np.isfinite(quaternion))
                or float(np.linalg.norm(quaternion)) <= 1e-12
            ):
                raise ValueError(
                    'mocap quaternion must contain four finite values'
                )
        self._latest_mocap = packet
        if packet.data.get('position_forwarded_to_onboard_ekf') is False:
            return
        self._mocap_frames.append(packet)
        if self._ekf is not None:
            if len(self._pending_positions) == self._pending_positions.maxlen:
                self._skipped_position_packets += 1
            self._pending_positions.append(packet)

    @staticmethod
    def _mocap_measurement_host_time(packet):
        """Return the best honest host-clock time attached to a frame.

        Current Vicon integration exposes host time immediately after
        ``waitForNextFrame`` rather than a camera capture timestamp.  It is
        useful for bounded availability-time alignment, but it is deliberately
        not promoted to a device/capture clock.
        """
        timing = packet.data.get('mocap_timing')
        value = (
            timing.get('wait_return_monotonic_s')
            if hasattr(timing, 'get') else None
        )
        try:
            value = float(value)
        except (TypeError, ValueError):
            value = ContactAttitudeShadow._packet_monotonic_time(packet)
        if not math.isfinite(value):
            raise ValueError('mocap host time must be finite')
        return value

    def _position_history_match(self, packet, history):
        if packet.cf_timestamp_ms is not None:
            raw_timestamp = int(packet.cf_timestamp_ms)
            matches = [
                (index, entry) for index, entry in enumerate(history)
                if entry['cf_timestamp_ms'] == raw_timestamp
            ]
            if matches:
                index, _entry = matches[-1]
                return index, 0.0, 'cf_device_timestamp_exact', True, False
            latest = history[-1]['cf_timestamp_ms']
            offset_ms = self._timestamp_offset_ms(raw_timestamp, latest)
            return (
                None, None, 'cf_device_timestamp_exact', True,
                offset_ms > 0,
            )

        measurement_time = self._mocap_measurement_host_time(packet)
        latest_time = float(history[-1]['host_receive_monotonic_s'])
        if measurement_time > latest_time:
            return (
                None, None, 'host_after_wait_availability_approximation',
                False, True,
            )
        candidates = [
            (
                abs(1000.0 * (
                    float(entry['host_receive_monotonic_s']) - measurement_time
                )),
                index,
            )
            for index, entry in enumerate(history)
        ]
        skew_ms, index = min(candidates)
        if skew_ms > self.config.state_join_tolerance_ms:
            return (
                None, skew_ms,
                'host_after_wait_availability_approximation', False, False,
            )
        signed_skew_ms = 1000.0 * (
            measurement_time
            - float(history[index]['host_receive_monotonic_s'])
        )
        return (
            index, signed_skew_ms,
            'host_after_wait_availability_approximation', False, False,
        )

    def _apply_position_at_history_index(
            self, history, index, packet, deadline=None):
        """Build a candidate corrected history without partial mutation."""
        replay_span = len(history) - 1 - index
        if replay_span > self.config.max_position_replay_samples:
            return None, False, 'position_replay_sample_budget_exceeded'
        working = copy.deepcopy(history[index]['ekf'])
        before_updates = working.snapshot().position_update_count
        updated = working.update_extpos(self._mocap_position(packet))
        accepted = updated.position_update_count > before_updates
        candidate_history = list(history)
        candidate_history[index] = {
            **history[index], 'ekf': copy.deepcopy(working),
        }
        for later in range(index + 1, len(history)):
            if deadline is not None and self._perf_clock() > deadline:
                return None, False, 'position_replay_time_budget_exceeded'
            entry = history[later]
            propagated = working.propagate(
                entry['cf_timestamp_ms'],
                entry['gyro_deg_s'],
                entry['accel_g'],
            )
            if not propagated.valid:
                return None, False, 'position_repropagation_failed'
            candidate_history[later] = {
                **entry, 'ekf': copy.deepcopy(working),
            }
        return candidate_history, accepted, None

    def _fuse_pending_positions(self, deadline=None):
        if self._ekf is None or not self._pending_positions:
            return True
        if not self._ekf_history:
            return True
        history = list(self._ekf_history)
        future = []
        for packet in self._pending_positions:
            if deadline is not None and self._perf_clock() > deadline:
                return False
            index, skew_ms, basis, strict, is_future = (
                self._position_history_match(packet, history)
            )
            if is_future:
                future.append(packet)
                continue
            if index is None:
                self._last_position_measurement_skew_ms = skew_ms
                self._last_position_timing_basis = basis
                self._skipped_position_packets += 1
                continue
            if strict:
                if self._last_processed_position_cf_timestamp_ms is not None:
                    offset_ms = self._timestamp_offset_ms(
                        packet.cf_timestamp_ms,
                        self._last_processed_position_cf_timestamp_ms,
                    )
                    if offset_ms <= 0:
                        self._fatal_reason = self._fatal_reason or (
                            'position_measurement_epoch_nonmonotonic'
                        )
                        self._invalid_reason = self._fatal_reason
                        break
            else:
                measurement_time = self._mocap_measurement_host_time(packet)
                if (
                    self._last_processed_approximate_position_time_s
                    is not None
                    and measurement_time
                    <= self._last_processed_approximate_position_time_s
                ):
                    self._fatal_reason = self._fatal_reason or (
                        'position_measurement_epoch_nonmonotonic'
                    )
                    self._invalid_reason = self._fatal_reason
                    break
            candidate_history, accepted, replay_error = (
                self._apply_position_at_history_index(
                    history, index, packet, deadline=deadline
                )
            )
            if candidate_history is None:
                self._fatal_reason = self._fatal_reason or replay_error
                self._invalid_reason = self._fatal_reason
                break
            history = candidate_history
            if strict:
                self._last_processed_position_cf_timestamp_ms = (
                    packet.cf_timestamp_ms
                )
            else:
                self._last_processed_approximate_position_time_s = (
                    measurement_time
                )
            self._ekf = copy.deepcopy(history[-1]['ekf'])
            self._last_ekf_host_receive_monotonic_s = float(
                history[-1]['host_receive_monotonic_s']
            )
            self._ekf_history = collections.deque(
                history, maxlen=self.config.history_capacity
            )
            self._last_position_measurement_skew_ms = float(skew_ms)
            self._last_position_timing_basis = basis
            if accepted:
                if strict:
                    self._strict_position_time_update_count += 1
                    self._last_strict_position_update_cf_timestamp_ms = (
                        packet.cf_timestamp_ms
                    )
                else:
                    self._approximate_position_time_update_count += 1
            history = list(self._ekf_history)
        if self._fatal_reason is not None:
            # Do not retain measurements after the first deterministic replay
            # failure; the shadow is latched invalid.
            self._pending_positions.clear()
        else:
            self._pending_positions = collections.deque(future, maxlen=64)
        return True

    def _process_gyro(self, packet, joined=None):
        try:
            gyro = self._gyro(packet)
        except KeyError:
            self._fatal_reason = (
                self._fatal_reason or "gyro_packet_missing_field"
            )
            self._invalid_reason = "gyro_packet_missing_field"
            return
        self._latest_gyro = packet
        if self.observer.phase in (self.observer.ALIGNING, self.observer.READY):
            if joined is not None:
                acceleration, yaw = joined
                estimate = self.observer.add_alignment_sample(
                    packet.cf_timestamp_ms, acceleration, gyro, yaw
                )
                if estimate.valid:
                    self._invalid_reason = self._fatal_reason
            return
        if self.observer.phase == self.observer.CONTACT:
            estimate = self.observer.add_gyro_sample(
                packet.cf_timestamp_ms, gyro
            )
            if not estimate.valid:
                self._fatal_reason = self._fatal_reason or estimate.reason
                self._invalid_reason = estimate.reason
            self._contact_imu_history.append({
                'packet': packet,
                'gyro_deg_s': tuple(float(value) for value in gyro),
                'accel_g': (
                    None if joined is None else tuple(
                        float(value) for value in joined[0]
                    )
                ),
                'observer': estimate,
            })
            # During an unconfirmed recontact candidate, keep the released
            # EKF alive in parallel.  If the force spike cancels, the EKF has
            # a continuous IMU timeline; if contact confirms, it is discarded
            # before accelerometer-corrupted state can gain authority.
            if (
                self._contact_candidate_origin != 'post_release'
                or self._ekf is None
            ):
                return
        if self._ekf is not None:
            if joined is None:
                # `_flush_pending_gyros` owns the bounded reorder timeout.  This
                # branch is reachable during contact-only propagation, where
                # acceleration is intentionally not an attitude observation.
                return
            acceleration, _yaw = joined
            estimate = self._ekf.propagate(
                packet.cf_timestamp_ms, gyro, acceleration
            )
            if not estimate.valid:
                self._fatal_reason = self._fatal_reason or estimate.reason
                self._invalid_reason = estimate.reason
            else:
                self._last_ekf_host_receive_monotonic_s = (
                    self._packet_monotonic_time(packet)
                )
                self._ekf_history.append({
                    'cf_timestamp_ms': estimate.cf_timestamp_ms,
                    'unwrapped_timestamp_ms': estimate.unwrapped_timestamp_ms,
                    'host_receive_monotonic_s': (
                        self._packet_monotonic_time(packet)
                    ),
                    'gyro_deg_s': tuple(float(value) for value in gyro),
                    'accel_g': tuple(float(value) for value in acceleration),
                    'ekf': copy.deepcopy(self._ekf),
                })

    @_contain_shadow_errors("begin_contact")
    def begin_contact(self) -> dict:
        """Freeze accelerometer fusion at the first contact-threshold edge."""
        self.drain()
        if self._fatal_reason is not None:
            self._invalid_reason = self._fatal_reason
            return self.snapshot()
        if self.config.mode == ONBOARD_MIRROR:
            fresh, reason, _age = self._onboard_state_freshness()
            if not fresh:
                self._invalid_reason = reason
            elif self._mirror_phase not in ('tracking', 'released'):
                self._invalid_reason = 'onboard_mirror_contact_already_active'
            else:
                self._mirror_previous_phase = self._mirror_phase
                self._mirror_phase = 'contact_candidate'
                self._invalid_reason = self._fatal_reason
                self._contact_candidate_origin = 'onboard_mirror'
            return self.snapshot()
        if self.observer.phase == self.observer.READY:
            state_ready, state_reason = self._initial_alignment_state_gate(
                self.observer.snapshot()
            )
            if not state_ready:
                self._invalid_reason = state_reason
                return self.snapshot()
            gyro_fresh, gyro_reason, _gyro_age = (
                self._processed_gyro_freshness(
                    self.observer.snapshot().cf_timestamp_ms
                )
            )
            if not gyro_fresh:
                self._invalid_reason = gyro_reason
                return self.snapshot()
            self._pre_candidate_observer = copy.deepcopy(self.observer)
            self._contact_candidate_origin = 'alignment'
            estimate = self.observer.begin_contact()
            if estimate.valid:
                self._contact_imu_history.clear()
                if self._latest_gyro is not None:
                    joined = self._joined_accel(self._latest_gyro)
                    self._contact_imu_history.append({
                        'packet': self._latest_gyro,
                        'gyro_deg_s': tuple(
                            float(value) for value in self._gyro(
                                self._latest_gyro
                            )
                        ),
                        'accel_g': (
                            None if joined is None else tuple(
                                float(value) for value in joined[0]
                            )
                        ),
                        'observer': estimate,
                    })
        elif self._ekf is not None and self._latest_gyro is not None:
            ekf = self._ekf.snapshot()
            now = float(self._clock())
            gyro_fresh, gyro_reason, gyro_age_s = (
                self._processed_gyro_freshness(ekf.cf_timestamp_ms)
            )
            ekf_age_s = (
                None if self._last_ekf_host_receive_monotonic_s is None
                else now - self._last_ekf_host_receive_monotonic_s
            )
            if (
                not ekf.valid or not gyro_fresh
            ):
                self._contact_candidate_origin = None
                self._invalid_reason = (
                    'recontact_ekf_invalid'
                    if not ekf.valid else 'recontact_' + gyro_reason
                )
                return self.snapshot()
            if (
                gyro_age_s < -self.config.max_packet_host_skew_s
                or gyro_age_s > self.config.max_packet_host_age_s
                or ekf_age_s is None
                or ekf_age_s < -self.config.max_packet_host_skew_s
                or ekf_age_s > self.config.max_packet_host_age_s
            ):
                self._contact_candidate_origin = None
                self._invalid_reason = 'recontact_inertial_state_host_stale'
                return self.snapshot()
            self._pre_candidate_observer = copy.deepcopy(self.observer)
            try:
                gyro = self._gyro(self._latest_gyro)
            except KeyError:
                self._contact_candidate_origin = None
                self._invalid_reason = 'recontact_gyro_packet_missing_field'
                return self.snapshot()
            self._contact_candidate_origin = 'post_release'
            estimate = self.observer.begin_contact_from_state(
                quaternion_wxyz=ekf.quaternion_wxyz,
                gyro_bias_deg_s=[
                    value * 180.0 / 3.141592653589793
                    for value in ekf.gyro_bias_rad_s
                ],
                cf_timestamp_ms=ekf.cf_timestamp_ms,
                unwrapped_timestamp_ms=ekf.unwrapped_timestamp_ms,
                gyro_deg_s=gyro,
            )
            if estimate.valid:
                joined = self._joined_accel(self._latest_gyro)
                self._contact_imu_history.clear()
                self._contact_imu_history.append({
                    'packet': self._latest_gyro,
                    'gyro_deg_s': tuple(float(value) for value in gyro),
                    'accel_g': (
                        None if joined is None else tuple(
                            float(value) for value in joined[0]
                        )
                    ),
                    'observer': estimate,
                })
        else:
            self._contact_candidate_origin = None
            self._invalid_reason = 'contact_before_alignment_ready'
            return self.snapshot()
        if not estimate.valid:
            self._invalid_reason = estimate.reason
        else:
            self._invalid_reason = self._fatal_reason
        return self.snapshot()

    # Compatibility for synthetic callers from before the lifecycle name was
    # corrected. Production runtime uses begin_contact().
    begin_contact_candidate = begin_contact

    @_contain_shadow_errors("confirm_contact")
    def confirm_contact(self) -> dict:
        """Commit a threshold candidate after the force-onset dwell."""
        if self.config.mode == ONBOARD_MIRROR:
            if self._mirror_phase != 'contact_candidate':
                self._invalid_reason = 'onboard_mirror_confirm_without_candidate'
                return self.snapshot()
            if self._fatal_reason is not None:
                self._invalid_reason = self._fatal_reason
                return self.snapshot()
            fresh, reason, _age = self._onboard_state_freshness()
            if not fresh:
                self._invalid_reason = reason
                return self.snapshot()
            self._mirror_phase = 'contact'
            self._mirror_previous_phase = None
            self._contact_candidate_origin = None
            self._invalid_reason = self._fatal_reason
            return self.snapshot()
        if self._contact_candidate_origin not in ('alignment', 'post_release'):
            self._invalid_reason = 'confirm_without_contact_candidate'
            return self.snapshot()
        if self._contact_candidate_origin == 'post_release':
            self._ekf = None
            self._ekf_history.clear()
            self._pending_positions.clear()
            self._last_ekf_host_receive_monotonic_s = None
        self._contact_candidate_origin = None
        self._pre_candidate_observer = None
        self._invalid_reason = self._fatal_reason
        return self.snapshot()

    @_contain_shadow_errors("cancel_contact_candidate")
    def cancel_contact_candidate(self) -> dict:
        """Resume alignment/EKF processing after a false force threshold."""
        if self.config.mode == ONBOARD_MIRROR:
            if self._mirror_phase != 'contact_candidate':
                self._invalid_reason = 'onboard_mirror_cancel_without_candidate'
                return self.snapshot()
            self._mirror_phase = self._mirror_previous_phase or 'tracking'
            self._mirror_previous_phase = None
            self._contact_candidate_origin = None
            self._invalid_reason = self._fatal_reason
            return self.snapshot()
        if self._contact_candidate_origin not in ('alignment', 'post_release'):
            self._invalid_reason = 'cancel_without_contact_candidate'
            return self.snapshot()
        if (
            self._contact_candidate_origin == 'post_release'
            and self._pre_candidate_observer is not None
        ):
            # The released EKF was propagated in parallel throughout the
            # candidate, so restoring its prior observer shell is causal and
            # does not introduce an IMU timestamp gap.
            self.observer = self._pre_candidate_observer
        else:
            self.observer.reset()
            self._ekf = None
            self._release_snapshot = None
        self._contact_candidate_origin = None
        self._pre_candidate_observer = None
        self._contact_imu_history.clear()
        self._invalid_reason = self._fatal_reason
        return self.snapshot()

    cancel_initial_contact = cancel_contact_candidate

    def _select_release_state_pair(
            self, release_estimate, release_gyro_packet,
            release_epoch_host_monotonic_s):
        """Select causal release position, velocity, attitude, and bias seeds.

        Velocity comes from the atomic ``CONTACT_STATE_SEED`` packet, while
        attitude and gyro bias come from the gyro observer at the selected
        physical-release epoch. Position comes from the latest causal Vicon
        ``tvec`` actually forwarded to the onboard EKF. Live frames without a
        capture-to-CF clock map remain explicitly approximate; their quaternion
        is evaluation-only and never enters this observer.
        """
        seed_by_timestamp = {
            int(packet.cf_timestamp_ms): packet
            for packet in self._state_seed_packets
        }
        history_by_timestamp = {
            int(entry['observer'].cf_timestamp_ms): entry
            for entry in self._contact_imu_history
            if entry['observer'].cf_timestamp_ms is not None
        }
        candidates = []
        for timestamp in seed_by_timestamp.keys() & history_by_timestamp.keys():
            age_ms = self._timestamp_offset_ms(
                release_estimate.cf_timestamp_ms, timestamp
            )
            if 0 <= age_ms <= self.config.state_join_tolerance_ms:
                candidates.append((age_ms, timestamp))
        if not candidates:
            return None, "release_state_pair_missing_or_noncausal"
        state_age_ms, state_timestamp = min(
            candidates, key=lambda item: item[0]
        )
        seed_packet = seed_by_timestamp[state_timestamp]
        state_history_entry = history_by_timestamp[state_timestamp]
        gyro_packet = release_gyro_packet
        if (
            gyro_packet is None
            or gyro_packet.cf_timestamp_ms != release_estimate.cf_timestamp_ms
        ):
            return None, "release_gyro_epoch_mismatch"
        gyro_host_time = self._packet_monotonic_time(gyro_packet)
        release_epoch_host_monotonic_s = float(
            release_epoch_host_monotonic_s
        )
        if not math.isfinite(release_epoch_host_monotonic_s):
            return None, "release_event_monotonic_time_invalid"
        seed_host_time = self._packet_monotonic_time(seed_packet)

        position_candidates = []
        for packet in self._mocap_frames:
            if packet.data.get('position_forwarded_to_onboard_ekf') is not True:
                continue
            position = self._mocap_position(packet)
            measurement_host_time = self._mocap_measurement_host_time(packet)
            if packet.cf_timestamp_ms is not None:
                skew_ms = self._timestamp_offset_ms(
                    packet.cf_timestamp_ms,
                    release_estimate.cf_timestamp_ms,
                )
                if (
                    skew_ms > 0
                    or abs(skew_ms) > self.config.state_join_tolerance_ms
                ):
                    continue
                timing_basis = 'cf_device_timestamp_exact'
                timing_strict = True
                timing_priority = 0
            else:
                if measurement_host_time > release_epoch_host_monotonic_s:
                    continue
                skew_ms = 1000.0 * (
                    measurement_host_time - gyro_host_time
                )
                if abs(skew_ms) > 1000.0 * self.config.max_packet_host_skew_s:
                    continue
                timing_basis = 'host_after_wait_availability_approximation'
                timing_strict = False
                timing_priority = 1
            position_candidates.append((
                timing_priority,
                abs(float(skew_ms)),
                -float(measurement_host_time),
                packet,
                position,
                float(skew_ms),
                timing_basis,
                timing_strict,
                float(measurement_host_time),
            ))
        if not position_candidates:
            return None, 'release_extpos_position_missing_or_noncausal'
        (
            _timing_priority,
            _absolute_skew_ms,
            _negative_host_time,
            position_packet,
            synchronized_position,
            position_skew_ms,
            position_timing_basis,
            position_timing_strict,
            position_host_time,
        ) = min(position_candidates, key=lambda item: item[:3])
        now = float(self._clock())
        host_times = [seed_host_time, gyro_host_time, position_host_time]
        host_ages = [now - packet_time for packet_time in host_times]
        if any(
            age < -self.config.max_packet_host_skew_s
            or age > self.config.max_packet_host_age_s
            for age in host_ages
        ):
            return None, "release_state_packet_host_stale"
        if max(host_times) - min(host_times) > self.config.max_packet_host_skew_s:
            return None, 'release_state_packet_host_skew_exceeded'
        return {
            'seed_packet': seed_packet,
            'state_history_entry': state_history_entry,
            'state_age_ms': float(state_age_ms),
            'position_packet': position_packet,
            'position_m': synchronized_position,
            'position_to_gyro_skew_ms': position_skew_ms,
            'position_timing_basis': position_timing_basis,
            'position_timing_strict': position_timing_strict,
            'host_age_s': float(max(host_ages)),
            'host_skew_s': float(max(host_times) - min(host_times)),
        }, None

    def _release_event_time(self, release_event_monotonic_s, fallback):
        if release_event_monotonic_s is None:
            return float(fallback), 'implicit_latest_estimator_sample'
        value = float(release_event_monotonic_s)
        if not math.isfinite(value):
            raise ValueError('release event monotonic time must be finite')
        return value, 'force_sensor_candidate_onset_monotonic'

    def _select_release_imu_epoch(self, release_event_monotonic_s):
        if not self._contact_imu_history:
            return None, 'release_event_gyro_history_missing'
        fallback = self._packet_monotonic_time(
            self._contact_imu_history[-1]['packet']
        )
        event_time, event_source = self._release_event_time(
            release_event_monotonic_s, fallback
        )
        candidates = [
            (self._packet_monotonic_time(entry['packet']), index, entry)
            for index, entry in enumerate(self._contact_imu_history)
            if self._packet_monotonic_time(entry['packet']) <= event_time
        ]
        if not candidates:
            return None, 'release_event_gyro_epoch_unavailable'
        # Device-clock-only replays legitimately give every packet the same
        # placeholder host time. Break host-time ties by causal history order so
        # the newest gyro sample owns the implicit release epoch.
        gyro_time, _history_index, entry = max(
            candidates, key=lambda item: (item[0], item[1])
        )
        skew_s = gyro_time - event_time
        if abs(skew_s) > self.config.max_release_event_to_gyro_skew_s:
            return None, 'release_event_gyro_skew_exceeded'
        estimate = entry['observer']
        if (
            not estimate.valid
            or estimate.phase != self.observer.CONTACT
            or estimate.quaternion_wxyz is None
            or estimate.gyro_bias_deg_s is None
        ):
            return None, 'release_event_gyro_estimate_invalid'
        return {
            'estimate': copy.deepcopy(estimate),
            'gyro_packet': entry['packet'],
            'gyro_deg_s': tuple(entry['gyro_deg_s']),
            'accel_g': (
                None if entry['accel_g'] is None
                else tuple(entry['accel_g'])
            ),
            'event_monotonic_s': event_time,
            'event_source': event_source,
            'event_to_gyro_skew_s': float(skew_s),
        }, None

    def _select_mirror_release_state(self, release_event_monotonic_s):
        if not self._onboard_state_history:
            return None, 'onboard_mirror_release_state_missing'
        if release_event_monotonic_s is None:
            event_time = float(self._clock())
            event_source = 'implicit_release_call_monotonic'
        else:
            event_time, event_source = self._release_event_time(
                release_event_monotonic_s,
                self._onboard_state_history[-1][
                    'host_receive_monotonic_s'
                ],
            )
        candidates = [
            state for state in self._onboard_state_history
            if state['host_receive_monotonic_s'] <= event_time
        ]
        if not candidates:
            return None, 'onboard_mirror_release_epoch_unavailable'
        state = max(
            candidates, key=lambda item: item['host_receive_monotonic_s']
        )
        skew_s = state['host_receive_monotonic_s'] - event_time
        if abs(skew_s) > self.config.max_release_event_to_gyro_skew_s:
            return None, 'onboard_mirror_release_epoch_unavailable'
        return {
            'state': copy.deepcopy(state),
            'event_monotonic_s': event_time,
            'event_source': event_source,
            'event_to_state_skew_s': float(skew_s),
        }, None

    @_contain_shadow_errors("release")
    def release(
            self, position_m: Sequence[float], velocity_m_s: Sequence[float],
            *, interaction_direction=None, interaction_direction_source=None,
            active_setpoint=None, effective_command_at_state=None,
            pending_transport_commands=None,
            inner_loop_tail=None,
            release_event_monotonic_s=None,
    ) -> dict:
        release_started = self._perf_clock()
        release_deadline = (
            release_started + self.config.max_release_processing_time_s
        )
        self.drain()
        if self._perf_clock() > release_deadline:
            self._release_budget_exceeded_count += 1
            self._fatal_reason = self._fatal_reason or (
                'release_processing_time_budget_exceeded'
            )
            self._invalid_reason = self._fatal_reason
            self._pending_release = None
            return self.snapshot()
        if self._fatal_reason is not None:
            self._invalid_reason = self._fatal_reason
            return self.snapshot()
        if self.config.mode == ONBOARD_MIRROR:
            selected, reason = self._select_mirror_release_state(
                release_event_monotonic_s
            )
            if selected is None:
                self._fatal_reason = self._fatal_reason or reason
                self._invalid_reason = self._fatal_reason
                return self.snapshot()
            if self._mirror_phase != 'contact':
                self._invalid_reason = 'onboard_mirror_release_outside_contact'
                return self.snapshot()
            release_state = selected['state']
            self._mirror_phase = 'released'
            self._mirror_previous_phase = None
            self._contact_candidate_origin = None
            self._release_snapshot = {
                'position_m': copy.deepcopy(
                    release_state['position_m']
                ),
                'onboard_ekf_velocity_m_s': copy.deepcopy(
                    release_state['velocity_m_s']
                ),
                'gyro_quaternion_wxyz': None,
                'gyro_bias_deg_s': None,
                'position_source': 'onboard_ekf_mirror',
                'velocity_source': 'onboard_ekf_mirror',
                'state_time_s': release_state['state_time_s'],
                'state_host_age_s': -selected['event_to_state_skew_s'],
                'state_sequence': release_state['sequence'],
                'release_event_monotonic_s': selected[
                    'event_monotonic_s'
                ],
                'release_event_time_source': selected['event_source'],
                'release_event_to_state_skew_s': selected[
                    'event_to_state_skew_s'
                ],
                'command_history_used_for_state_reconstruction': False,
            }
            self._invalid_reason = self._fatal_reason
            return self.snapshot()
        current_preview = self.observer.snapshot()
        if self._contact_candidate_origin is not None:
            self._invalid_reason = 'release_before_contact_confirmation'
            return self.snapshot()
        if (
            not current_preview.valid
            or current_preview.phase != self.observer.CONTACT
            or current_preview.quaternion_wxyz is None
            or current_preview.gyro_bias_deg_s is None
        ):
            self._invalid_reason = "release_outside_valid_contact"
            return self.snapshot()
        if self._pending_release is None:
            # Freeze the first confirmed physical-release epoch. A later retry
            # may wait for an independently delivered state log block, but it
            # must never relabel that later callback time as the release time.
            host_loop_position = np.asarray(position_m, dtype=float)
            host_loop_velocity = np.asarray(velocity_m_s, dtype=float)
            if (
                host_loop_position.shape != (3,)
                or host_loop_velocity.shape != (3,)
                or not np.all(np.isfinite(host_loop_position))
                or not np.all(np.isfinite(host_loop_velocity))
            ):
                self._invalid_reason = 'release_host_loop_state_invalid'
                return self.snapshot()
            if interaction_direction is not None:
                synchronized_direction = np.asarray(
                    interaction_direction, dtype=float
                )
                if (
                    synchronized_direction.shape != (3,)
                    or not np.all(np.isfinite(synchronized_direction))
                ):
                    self._invalid_reason = (
                        'release_interaction_direction_invalid'
                    )
                    return self.snapshot()
                synchronized_direction = synchronized_direction.tolist()
            else:
                synchronized_direction = None
            selected_release, release_epoch_error = (
                self._select_release_imu_epoch(release_event_monotonic_s)
            )
            if selected_release is None:
                self._fatal_reason = (
                    self._fatal_reason or release_epoch_error
                )
                self._invalid_reason = self._fatal_reason
                return self.snapshot()
            self._pending_release = {
                'estimate': selected_release['estimate'],
                'gyro_packet': selected_release['gyro_packet'],
                'release_gyro_deg_s': selected_release['gyro_deg_s'],
                'release_accel_g': selected_release['accel_g'],
                'release_epoch_host_monotonic_s': selected_release[
                    'event_monotonic_s'
                ],
                'release_event_time_source': selected_release[
                    'event_source'
                ],
                'release_event_to_gyro_skew_s': selected_release[
                    'event_to_gyro_skew_s'
                ],
                'host_loop_position': host_loop_position.tolist(),
                'host_loop_velocity': host_loop_velocity.tolist(),
                'interaction_direction': synchronized_direction,
                'interaction_direction_source': copy.deepcopy(
                    interaction_direction_source
                ),
                'active_setpoint': copy.deepcopy(active_setpoint),
                'effective_command_at_state': copy.deepcopy(
                    effective_command_at_state
                ),
                'pending_transport_commands': copy.deepcopy(
                    pending_transport_commands or []
                ),
                'inner_loop_tail': copy.deepcopy(inner_loop_tail),
            }
        transaction = self._pending_release
        preview = transaction['estimate']
        release_gyro_packet = transaction['gyro_packet']
        state_pair, state_error = self._select_release_state_pair(
            preview,
            release_gyro_packet,
            transaction['release_epoch_host_monotonic_s'],
        )
        if state_error is not None:
            self._invalid_reason = state_error
            return self.snapshot()
        seed_packet = state_pair['seed_packet']
        position_packet = state_pair['position_packet']
        synchronized_position = state_pair['position_m']
        onboard_position = self._position(seed_packet)
        synchronized_velocity = self._velocity(seed_packet)
        host_loop_position = np.asarray(
            transaction['host_loop_position'], dtype=float
        )
        host_loop_velocity = np.asarray(
            transaction['host_loop_velocity'], dtype=float
        )
        synchronized_direction = transaction['interaction_direction']
        interaction_direction_source = transaction[
            'interaction_direction_source'
        ]
        copied_active_setpoint = transaction['active_setpoint']
        copied_effective_command = transaction['effective_command_at_state']
        copied_pending_commands = transaction['pending_transport_commands']
        copied_inner_loop_tail = transaction['inner_loop_tail']
        release_gyro_deg_s = transaction['release_gyro_deg_s']
        release_accel_g = transaction['release_accel_g']
        if release_accel_g is None:
            self._invalid_reason = 'release_epoch_accel_missing'
            return self.snapshot()
        candidate_ekf = PostReleaseInertialEkf(
            position_m=synchronized_position,
            velocity_m_s=synchronized_velocity,
            quaternion_wxyz=preview.quaternion_wxyz,
            gyro_bias_rad_s=[
                value * 3.141592653589793 / 180.0
                for value in preview.gyro_bias_deg_s
            ],
            cf_timestamp_ms=preview.cf_timestamp_ms,
            unwrapped_timestamp_ms=preview.unwrapped_timestamp_ms,
            initial_gyro_deg_s=release_gyro_deg_s,
            config=self.ekf_config,
        )
        initial_covariance_diagonal = [
            float(value) for value in np.diag(candidate_ekf.covariance)
        ]
        release_candidate = candidate_ekf.snapshot()
        replay_history = [{
            'cf_timestamp_ms': release_candidate.cf_timestamp_ms,
            'unwrapped_timestamp_ms': release_candidate.unwrapped_timestamp_ms,
            'host_receive_monotonic_s': self._packet_monotonic_time(
                release_gyro_packet
            ),
            'gyro_deg_s': release_gyro_deg_s,
            'accel_g': release_accel_g,
            'ekf': copy.deepcopy(candidate_ekf),
        }]
        replayed_imu_count = 0
        for entry in self._contact_imu_history:
            estimate = entry['observer']
            if estimate.unwrapped_timestamp_ms <= preview.unwrapped_timestamp_ms:
                continue
            if entry['accel_g'] is None:
                self._invalid_reason = 'release_replay_accel_missing'
                return self.snapshot()
            if replayed_imu_count >= self.config.max_release_replay_samples:
                self._release_budget_exceeded_count += 1
                self._fatal_reason = self._fatal_reason or (
                    'release_replay_sample_budget_exceeded'
                )
                self._invalid_reason = self._fatal_reason
                self._pending_release = None
                return self.snapshot()
            if self._perf_clock() > release_deadline:
                self._release_budget_exceeded_count += 1
                self._fatal_reason = self._fatal_reason or (
                    'release_processing_time_budget_exceeded'
                )
                self._invalid_reason = self._fatal_reason
                self._pending_release = None
                return self.snapshot()
            propagated = candidate_ekf.propagate(
                estimate.cf_timestamp_ms,
                entry['gyro_deg_s'],
                entry['accel_g'],
            )
            if not propagated.valid:
                self._invalid_reason = (
                    'release_replay_' + propagated.reason
                )
                return self.snapshot()
            replayed_imu_count += 1
            replay_history.append({
                'cf_timestamp_ms': propagated.cf_timestamp_ms,
                'unwrapped_timestamp_ms': (
                    propagated.unwrapped_timestamp_ms
                ),
                'host_receive_monotonic_s': (
                    self._packet_monotonic_time(entry['packet'])
                ),
                'gyro_deg_s': entry['gyro_deg_s'],
                'accel_g': entry['accel_g'],
                'ekf': copy.deepcopy(candidate_ekf),
            })
        candidate = candidate_ekf.snapshot()
        if release_candidate is None:
            self._invalid_reason = 'release_replay_did_not_reach_gyro_epoch'
            return self.snapshot()
        if (
            candidate.unwrapped_timestamp_ms
            != current_preview.unwrapped_timestamp_ms
        ):
            self._invalid_reason = 'release_replay_did_not_reach_current_epoch'
            return self.snapshot()
        attitude_dot = min(1.0, abs(float(np.dot(
            release_candidate.quaternion_wxyz, preview.quaternion_wxyz
        ))))
        replay_attitude_difference_deg = math.degrees(
            2.0 * math.acos(attitude_dot)
        )
        if replay_attitude_difference_deg > (
                self.config.release_attitude_continuity_max_deg):
            self._invalid_reason = 'release_attitude_continuity_exceeded'
            return self.snapshot()
        if not replay_history:
            self._invalid_reason = 'release_replay_history_missing'
            return self.snapshot()
        if self._perf_clock() > release_deadline:
            self._release_budget_exceeded_count += 1
            self._fatal_reason = self._fatal_reason or (
                'release_processing_time_budget_exceeded'
            )
            self._invalid_reason = self._fatal_reason
            self._pending_release = None
            return self.snapshot()
        released = self.observer.release()
        if not released.valid:
            self._invalid_reason = released.reason
            return self.snapshot()
        self._ekf = candidate_ekf
        self._contact_candidate_origin = None
        self._pre_candidate_observer = None
        self._pending_positions.clear()
        self._ekf_history = collections.deque(
            replay_history, maxlen=self.config.history_capacity
        )
        self._last_ekf_host_receive_monotonic_s = float(
            self._ekf_history[-1]['host_receive_monotonic_s']
        )
        release_host_time = transaction['release_epoch_host_monotonic_s']
        for packet in self._mocap_frames:
            if packet.data.get('position_forwarded_to_onboard_ekf') is False:
                continue
            if packet.cf_timestamp_ms is not None:
                after_release = self._timestamp_offset_ms(
                    packet.cf_timestamp_ms, preview.cf_timestamp_ms
                ) > 0
            else:
                after_release = (
                    self._mocap_measurement_host_time(packet)
                    > release_host_time
                )
            if after_release:
                self._pending_positions.append(packet)
        self._invalid_reason = self._fatal_reason
        self._release_snapshot = {
            'position_m': [
                float(value) for value in release_candidate.position_m
            ],
            'release_velocity_m_s': [
                float(value) for value in release_candidate.velocity_m_s
            ],
            'external_position_seed_m': [
                float(value) for value in synchronized_position
            ],
            'initial_position_seed_m': [
                float(value) for value in synchronized_position
            ],
            'onboard_ekf_position_at_velocity_epoch_m': [
                float(value) for value in onboard_position
            ],
            'onboard_ekf_velocity_m_s': [
                float(value) for value in synchronized_velocity
            ],
            'position_source': 'raw_vicon_tvec_forwarded_to_onboard_ekf',
            'position_forward_route': (
                'extpose_position_component'
                if self.config.vicon_orientation_forwarded
                else 'extpos_position_only'
            ),
            'velocity_source': 'onboard_ekf_velocity_common_cf_epoch',
            'initial_state_is_independent_truth': False,
            'post_release_position_observation_source': (
                'raw_vicon_tvec_position_only'
            ),
            'position_seed_skew_ms': state_pair['position_to_gyro_skew_ms'],
            'position_seed_timing_basis': state_pair['position_timing_basis'],
            'position_seed_scientifically_time_aligned': state_pair[
                'position_timing_strict'
            ],
            'velocity_seed_skew_ms': -state_pair['state_age_ms'],
            'vicon_orientation_forwarded_to_onboard_ekf': (
                self.config.vicon_orientation_forwarded
            ),
            'state_seed_host_age_s': state_pair['host_age_s'],
            'state_seed_host_skew_s': state_pair['host_skew_s'],
            'host_loop_position_m': host_loop_position.tolist(),
            'host_loop_velocity_m_s': host_loop_velocity.tolist(),
            'state_seed_packet_sequence': seed_packet.sequence,
            'state_seed_cf_timestamp_ms': seed_packet.cf_timestamp_ms,
            'position_seed_packet_sequence': position_packet.sequence,
            'position_seed_frame_sequence': position_packet.sequence,
            'position_seed_cf_timestamp_ms': position_packet.cf_timestamp_ms,
            'position_seed_source_time_s': position_packet.data.get('time'),
            'position_seed_host_receive_time_s': float(
                position_packet.host_receive_time_s
            ),
            'release_gyro_packet_sequence': release_gyro_packet.sequence,
            'state_seed_packet_host_receive_time_s': float(
                seed_packet.host_receive_time_s
            ),
            'release_gyro_host_receive_time_s': float(
                release_gyro_packet.host_receive_time_s
            ),
            'release_replay_attitude_difference_deg': (
                replay_attitude_difference_deg
            ),
            'initial_covariance_diagonal': initial_covariance_diagonal,
            'gyro_quaternion_wxyz': list(preview.quaternion_wxyz),
            'gyro_legacy_body_rate_rad_s': (
                None
                if preview.legacy_body_rate_rad_s is None
                else list(preview.legacy_body_rate_rad_s)
            ),
            'gyro_bias_deg_s': list(preview.gyro_bias_deg_s),
            'interaction_direction': (
                synchronized_direction
            ),
            'interaction_direction_source': interaction_direction_source,
            'active_setpoint': copied_active_setpoint,
            'effective_command_at_state': copied_effective_command,
            # Only commands too recent to have reached the sampled state are
            # retained. Already-realized attitude is represented by q, not by
            # replaying old commands.
            'pending_transport_commands': copied_pending_commands,
            'pending_command_scope': (
                'recent_commands_not_confirmed_as_transport_queue'
            ),
            'inner_loop_tail': copied_inner_loop_tail,
            'cf_timestamp_ms': preview.cf_timestamp_ms,
            'unwrapped_timestamp_ms': preview.unwrapped_timestamp_ms,
            'ekf_commit_cf_timestamp_ms': candidate.cf_timestamp_ms,
            'ekf_commit_unwrapped_timestamp_ms': (
                candidate.unwrapped_timestamp_ms
            ),
            'release_to_commit_replay_ms': float(
                candidate.unwrapped_timestamp_ms
                - preview.unwrapped_timestamp_ms
            ),
            'release_replayed_imu_count': replayed_imu_count,
            'release_event_monotonic_s': transaction[
                'release_epoch_host_monotonic_s'
            ],
            'release_event_time_source': transaction[
                'release_event_time_source'
            ],
            'release_event_to_gyro_skew_s': transaction[
                'release_event_to_gyro_skew_s'
            ],
        }
        self._pending_release = None
        return self.snapshot()

    @_contain_shadow_errors("abandon_release_transaction")
    def abandon_release_transaction(self, reason='external_timeout') -> dict:
        """Fail closed when a frozen release epoch cannot be completed."""
        reason = str(reason)
        failure = f'release_transaction_abandoned:{reason}'
        self._pending_release = None
        self._fatal_reason = self._fatal_reason or failure
        self._invalid_reason = failure
        return self.snapshot()

    @_contain_shadow_errors("update_extpos")
    def update_extpos(self, position_m: Sequence[float], std_m=None) -> dict:
        """Fuse an explicitly time-aligned offline position observation.

        Production runtime queues the raw Vicon ``tvec`` after it has been
        forwarded to the Crazyflie. This method remains for deterministic
        synthetic tests and offline replay with an already aligned sample.
        """
        if self._ekf is not None:
            self._ekf.update_extpos(position_m, std_m=std_m)
        return self.snapshot()

    @staticmethod
    def _wrapped_angle_error_deg(left, right):
        return (float(left) - float(right) + 180.0) % 360.0 - 180.0

    def _mocap_snapshot(self):
        packet = self._latest_mocap
        if packet is None:
            return None
        data = packet.data
        result = {
            'source': 'vicon_rigidbody_or_point_position',
            'frame_id': data.get('frame_id'),
            'frame_sequence': packet.sequence,
            'source_time_s': data.get('time'),
            'host_receive_time_s': float(packet.host_receive_time_s),
            'position_m': self._mocap_position(packet),
            'position_forwarded_to_onboard_ekf': data.get(
                'position_forwarded_to_onboard_ekf'
            ),
            'orientation_forwarded_to_onboard_ekf': data.get(
                'orientation_forwarded_to_onboard_ekf',
                self.config.vicon_orientation_forwarded,
            ),
            'orientation_used_by_shadow_ekf': False,
            'quaternion_xyzw': None,
            'quaternion_wxyz': None,
            'legacy_rpy_deg': None,
        }
        quaternion = data.get('quat')
        if quaternion is None:
            return result
        x, y, z, w = np.asarray(quaternion, dtype=float)
        quaternion_wxyz = np.asarray([w, x, y, z], dtype=float)
        quaternion_wxyz /= np.linalg.norm(quaternion_wxyz)
        if quaternion_wxyz[0] < 0.0:
            quaternion_wxyz *= -1.0
        result['quaternion_xyzw'] = [float(x), float(y), float(z), float(w)]
        result['quaternion_wxyz'] = quaternion_wxyz.tolist()
        result['legacy_rpy_deg'] = np.degrees(
            legacy_rpy_from_quaternion(quaternion_wxyz)
        ).tolist()
        return result

    def _shadow_estimate(self, observer, ekf):
        if self.config.mode == ONBOARD_MIRROR:
            if self._latest_onboard_state is None:
                return None
            estimate = copy.deepcopy(self._latest_onboard_state)
            estimate.update({
                'source': 'onboard_ekf_mirror',
                'phase': self._mirror_phase,
                'copied_without_reestimation': True,
            })
            return estimate
        source = 'post_release_inertial_ekf' if ekf is not None else 'contact_gyro_observer'
        raw = ekf if ekf is not None else observer
        quaternion = raw.get('quaternion_wxyz')
        if quaternion is None:
            return None
        legacy_rpy = legacy_rpy_from_quaternion(quaternion)
        return {
            'source': source,
            'phase': 'post_release' if ekf is not None else observer['phase'],
            'cf_timestamp_ms': raw.get('cf_timestamp_ms'),
            'unwrapped_timestamp_ms': raw.get('unwrapped_timestamp_ms'),
            'position_m': None if ekf is None else ekf['position_m'],
            'velocity_m_s': None if ekf is None else ekf['velocity_m_s'],
            'quaternion_wxyz': list(quaternion),
            'legacy_rpy_rad': legacy_rpy.tolist(),
            'legacy_rpy_deg': np.degrees(legacy_rpy).tolist(),
            'copied_without_reestimation': False,
        }

    def _comparison_snapshot(self, shadow_estimate, mocap):
        onboard = self._latest_onboard_state
        comparison = {
            'onboard_minus_shadow_roll_pitch_deg': None,
            'vicon_minus_onboard_roll_pitch_deg': None,
            'vicon_minus_shadow_roll_pitch_deg': None,
            'onboard_shadow_mirror_exact': None,
            'vicon_orientation_withheld_from_onboard_estimator': None,
            'vicon_orientation_withheld_from_shadow_estimator': True,
            'shared_mocap_position_sensor_correlation_remains': True,
            'vicon_to_onboard_host_availability_skew_s': None,
            'comparison_time_aligned': False,
            'comparison_time_basis': None,
            'comparison_scientifically_valid': False,
        }
        if onboard is not None and shadow_estimate is not None:
            onboard_rp = onboard['legacy_rpy_deg'][:2]
            shadow_rp = shadow_estimate['legacy_rpy_deg'][:2]
            comparison['onboard_minus_shadow_roll_pitch_deg'] = [
                self._wrapped_angle_error_deg(left, right)
                for left, right in zip(onboard_rp, shadow_rp)
            ]
            if self.config.mode == ONBOARD_MIRROR:
                comparison['onboard_shadow_mirror_exact'] = (
                    onboard['position_m'] == shadow_estimate['position_m']
                    and onboard['velocity_m_s'] == shadow_estimate['velocity_m_s']
                    and onboard['legacy_rpy_rad']
                    == shadow_estimate['legacy_rpy_rad']
                )
        if mocap is None or mocap.get('legacy_rpy_deg') is None:
            return comparison
        if onboard is not None:
            skew_s = (
                self._mocap_measurement_host_time(self._latest_mocap)
                - float(onboard['host_receive_monotonic_s'])
            )
            comparison['vicon_to_onboard_host_availability_skew_s'] = skew_s
            comparison['comparison_time_basis'] = (
                'host_after_wait_availability_approximation'
            )
            comparison['comparison_time_aligned'] = (
                abs(skew_s) <= self.config.max_packet_host_skew_s
            )
            if not comparison['comparison_time_aligned']:
                return comparison
        forwarded = mocap.get('orientation_forwarded_to_onboard_ekf')
        comparison['vicon_orientation_withheld_from_onboard_estimator'] = (
            None if forwarded is None else not bool(forwarded)
        )
        if onboard is not None:
            comparison['vicon_minus_onboard_roll_pitch_deg'] = [
                self._wrapped_angle_error_deg(left, right)
                for left, right in zip(
                    mocap['legacy_rpy_deg'][:2],
                    onboard['legacy_rpy_deg'][:2],
                )
            ]
        if shadow_estimate is not None:
            comparison['vicon_minus_shadow_roll_pitch_deg'] = [
                self._wrapped_angle_error_deg(left, right)
                for left, right in zip(
                    mocap['legacy_rpy_deg'][:2],
                    shadow_estimate['legacy_rpy_deg'][:2],
                )
            ]
        return comparison

    def snapshot(self) -> dict:
        observer = asdict(self.observer.snapshot())
        ekf = None if self._ekf is None else asdict(self._ekf.snapshot())
        mocap = self._mocap_snapshot()
        shadow_estimate = self._shadow_estimate(observer, ekf)
        comparison = self._comparison_snapshot(shadow_estimate, mocap)
        mirror_fresh, mirror_reason, mirror_age_s = (
            self._onboard_state_freshness()
        )
        estimator_valid = (
            mirror_fresh if self.config.mode == ONBOARD_MIRROR
            else observer['valid']
        )
        effective_invalid_reason = self._invalid_reason
        if (
            effective_invalid_reason is None
            and self.config.mode == ONBOARD_MIRROR
            and not mirror_fresh
        ):
            effective_invalid_reason = mirror_reason
        initial_yaw_uses_forwarded_vicon = bool(
            self.config.alignment_legacy_yaw_deg is None
            and self.config.vicon_orientation_forwarded is True
        )
        position_timing_scientifically_valid = None
        if (
            self._strict_position_time_update_count
            or self._approximate_position_time_update_count
        ):
            position_timing_scientifically_valid = (
                self._strict_position_time_update_count > 0
                and self._approximate_position_time_update_count == 0
            )
        return {
            'shadow_only': True,
            'command_authority': False,
            'mode': self.config.mode,
            'experiment_run': self.config.experiment_run,
            'alignment_yaw_source': (
                'onboard_ekf_yaw'
                if self.config.alignment_legacy_yaw_deg is None
                else 'configured_nominal_yaw'
            ),
            'alignment_gate_metrics': copy.deepcopy(
                self._alignment_gate_metrics
            ),
            'valid': (
                self._fatal_reason is None
                and effective_invalid_reason is None
                and estimator_valid
            ),
            'invalid_reason': effective_invalid_reason,
            'fatal_reason': self._fatal_reason,
            'observer': observer,
            'post_release_ekf': ekf,
            'shadow_estimate': shadow_estimate,
            'onboard_ekf': copy.deepcopy(self._latest_onboard_state),
            'vicon': mocap,
            'comparison': comparison,
            'vicon_full_quaternion_used_by_shadow_ekf': False,
            'vicon_orientation_used_for_initial_yaw': (
                initial_yaw_uses_forwarded_vicon
            ),
            'vicon_orientation_used_by_shadow_ekf': (
                initial_yaw_uses_forwarded_vicon
            ),
            'command_history_used_for_state_reconstruction': False,
            'release_snapshot': copy.deepcopy(self._release_snapshot),
            'pending_release_cf_timestamp_ms': (
                None
                if self._pending_release is None
                else self._pending_release['estimate'].cf_timestamp_ms
            ),
            'dropped_packets': self._dropped_packets,
            'skipped_position_packets': self._skipped_position_packets,
            'last_position_measurement_skew_ms': (
                self._last_position_measurement_skew_ms
            ),
            'last_position_timing_basis': self._last_position_timing_basis,
            'strict_position_time_update_count': (
                self._strict_position_time_update_count
            ),
            'last_strict_position_update_cf_timestamp_ms': (
                self._last_strict_position_update_cf_timestamp_ms
            ),
            'approximate_position_time_update_count': (
                self._approximate_position_time_update_count
            ),
            'position_timing_scientifically_valid': (
                position_timing_scientifically_valid
            ),
            'onboard_state_fresh': mirror_fresh,
            'onboard_state_host_age_s': mirror_age_s,
            'contained_failure_count': self._failure_count,
            'last_drain_duration_s': self._last_drain_duration_s,
            'max_drain_duration_s': self._max_drain_duration_s,
            'drain_budget_exceeded_count': self._drain_budget_exceeded_count,
            'release_budget_exceeded_count': (
                self._release_budget_exceeded_count
            ),
            'queued_packets': len(self._queue) + len(self._pending_gyros),
            'pending_gyro_packets': len(self._pending_gyros),
            'contact_candidate_origin': self._contact_candidate_origin,
        }
