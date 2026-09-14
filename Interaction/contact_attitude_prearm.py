"""Pre-arm ownership for the contact-attitude shadow data path."""

from __future__ import annotations

import math
import threading

from Interaction.contact_attitude_experiment import (
    INERTIAL_POSITION,
    ONBOARD_MIRROR,
)
from Interaction.log_manager import CONTACT_SOURCE_TIMESTAMP_BASIS


class ContactAttitudePrearmHandle:
    """Register shadow listeners before arm without buffering flight data yet."""

    _REQUIRED_GROUPS = {
        ONBOARD_MIRROR: frozenset({'VEL_ORI', 'POS_ACC'}),
        INERTIAL_POSITION: frozenset({'GYRO_1KHZ'}),
    }

    def __init__(
            self, *, shadow, log_manager, mode, experiment_run,
            vicon_orientation_forwarded):
        if mode not in self._REQUIRED_GROUPS:
            raise ValueError('unsupported contact-attitude pre-arm mode')
        self.shadow = shadow
        self.log_manager = log_manager
        self.mode = mode
        self.experiment_run = int(experiment_run)
        self.vicon_orientation_forwarded = bool(
            vicon_orientation_forwarded
        )
        self.required_cf_groups = self._REQUIRED_GROUPS[mode]
        self._seen_cf_groups = set()
        self._last_cf_group_monotonic_s = {}
        self._invalid_cf_provenance = {}
        self._last_mocap_monotonic_s = None
        self._last_mocap_data = None
        self._lock = threading.Lock()
        self._active = False
        self._verified = False
        self._closed = False
        self._accepting_callbacks = True
        self._close_in_progress = False
        self._unsubscribers = []

    @classmethod
    def build(
            cls, *, log_manager, mode, experiment_run,
            vicon_orientation_forwarded, alignment_yaw_deg):
        """Construct the observer and atomically register both listeners."""
        if log_manager is None:
            raise RuntimeError(
                'contact-attitude experiment requires an active logger'
            )
        from Interaction.contact_attitude_shadow import (
            ContactAttitudeShadow,
            ContactAttitudeShadowConfig,
        )

        shadow = ContactAttitudeShadow(
            config=ContactAttitudeShadowConfig(
                mode=mode,
                experiment_run=experiment_run,
                vicon_orientation_forwarded=vicon_orientation_forwarded,
                alignment_legacy_yaw_deg=alignment_yaw_deg,
            ),
            report=lambda record: log_manager.add_log_entry(
                'contact_attitude_shadow', record
            ),
        )
        handle = cls(
            shadow=shadow,
            log_manager=log_manager,
            mode=mode,
            experiment_run=experiment_run,
            vicon_orientation_forwarded=vicon_orientation_forwarded,
        )
        log_manager.add_log_group('contact_attitude_shadow')
        try:
            handle._unsubscribers.append(
                log_manager.add_cf_packet_listener(handle.receive_cf_packet)
            )
            handle._unsubscribers.append(
                log_manager.add_mocap_frame_listener(
                    handle.receive_mocap_frame
                )
            )
        except Exception:
            handle.close()
            raise
        return handle

    def receive_cf_packet(self, packet):
        with self._lock:
            if not self._accepting_callbacks:
                return
            group = str(packet.group)
            provenance_required = (
                self.mode == INERTIAL_POSITION
                and self.experiment_run in (2, 3)
                and group in self.required_cf_groups
            )
            provenance_valid = (
                getattr(packet, 'source_snapshot_atomic', False) is True
                and getattr(packet, 'source_cf_timestamp_basis', None)
                == CONTACT_SOURCE_TIMESTAMP_BASIS
                and getattr(packet, 'transport_cf_timestamp_ms', None)
                is not None
            )
            if provenance_required and not provenance_valid:
                self._invalid_cf_provenance[group] = (
                    'producer-latched source timestamp is missing or invalid'
                )
            else:
                self._invalid_cf_provenance.pop(group, None)
                self._seen_cf_groups.add(group)
            timestamp = getattr(packet, 'host_receive_monotonic_s', None)
            try:
                timestamp = float(timestamp)
            except (TypeError, ValueError):
                timestamp = None
            if (
                timestamp is not None
                and math.isfinite(timestamp)
                and (not provenance_required or provenance_valid)
            ):
                self._last_cf_group_monotonic_s[group] = timestamp
            if self._active:
                # Serialize this short bounded enqueue with close(), so no
                # callback can append after cleanup has disabled delivery.
                self.shadow.enqueue_packet(packet)

    def receive_mocap_frame(self, packet):
        with self._lock:
            if not self._accepting_callbacks:
                return
            timestamp = getattr(packet, 'host_receive_monotonic_s', None)
            try:
                timestamp = float(timestamp)
            except (TypeError, ValueError):
                timestamp = None
            if timestamp is not None and math.isfinite(timestamp):
                self._last_mocap_monotonic_s = timestamp
            self._last_mocap_data = dict(packet.data)
            if self._active:
                self.shadow.enqueue_mocap_frame(packet)

    def missing_seen_groups(self):
        with self._lock:
            return tuple(sorted(
                self.required_cf_groups - self._seen_cf_groups
            ))

    def mark_verified(self):
        with self._lock:
            if self._closed:
                raise RuntimeError(
                    'contact-attitude pre-arm listeners are closed'
                )
            missing = self.required_cf_groups - self._seen_cf_groups
            invalid_provenance = tuple(sorted(
                group for group in self.required_cf_groups
                if group in self._invalid_cf_provenance
            ))
            if invalid_provenance:
                raise RuntimeError(
                    'contact-attitude listener rejected unproven firmware '
                    'source timestamps: ' + ', '.join(invalid_provenance)
                )
            if missing:
                raise RuntimeError(
                    'contact-attitude listener has not observed: '
                    + ', '.join(sorted(missing))
                )
            self._verified = True

    @staticmethod
    def _finite_vector(value, length):
        if not isinstance(value, (list, tuple)) or len(value) != length:
            return False
        try:
            return all(math.isfinite(float(item)) for item in value)
        except (TypeError, ValueError):
            return False

    def assert_fresh(self, *, now_s, max_age_s):
        """Fail closed unless the exact registered inputs are fresh now.

        This proves only host-side arrival freshness and route metadata. It
        does not claim that a Vicon capture has been mapped to the Crazyflie
        device clock.
        """
        try:
            now_s = float(now_s)
            max_age_s = float(max_age_s)
        except (TypeError, ValueError):
            raise ValueError('pre-arm freshness limits must be numeric') from None
        if not math.isfinite(now_s) or not math.isfinite(max_age_s):
            raise ValueError('pre-arm freshness limits must be finite')
        if max_age_s <= 0.0:
            raise ValueError('pre-arm max age must be positive')
        with self._lock:
            if self._closed or not self._accepting_callbacks:
                raise RuntimeError(
                    'contact-attitude pre-arm listeners are closed'
                )
            if not self._verified:
                raise RuntimeError(
                    'contact-attitude pre-arm readiness was not verified'
                )
            stale_groups = []
            for group in sorted(self.required_cf_groups):
                timestamp = self._last_cf_group_monotonic_s.get(group)
                age_s = None if timestamp is None else now_s - timestamp
                if (
                    age_s is None
                    or age_s < -max_age_s
                    or age_s > max_age_s
                ):
                    stale_groups.append(group)
            if stale_groups:
                raise RuntimeError(
                    'contact-attitude pre-arm CF streams are missing or stale: '
                    + ', '.join(stale_groups)
                )
            mocap_age_s = (
                None
                if self._last_mocap_monotonic_s is None
                else now_s - self._last_mocap_monotonic_s
            )
            if (
                mocap_age_s is None
                or mocap_age_s < -max_age_s
                or mocap_age_s > max_age_s
            ):
                raise RuntimeError(
                    'contact-attitude pre-arm Vicon stream is missing or stale'
                )
            frame = dict(self._last_mocap_data or {})
            expected_orientation_route = self.experiment_run == 3
            if (
                frame.get('position_forwarded_to_onboard_ekf') is not True
                or frame.get('orientation_forwarded_to_onboard_ekf')
                is not expected_orientation_route
            ):
                raise RuntimeError(
                    'contact-attitude pre-arm Vicon route is invalid'
                )
            if not self._finite_vector(frame.get('tvec'), 3):
                raise RuntimeError(
                    'contact-attitude pre-arm Vicon position is invalid'
                )
            if (
                self.experiment_run in (2, 3)
                and (
                    not self._finite_vector(frame.get('quat'), 4)
                    or math.sqrt(sum(
                        float(value) * float(value)
                        for value in frame.get('quat', ())
                    )) <= 1e-12
                )
            ):
                raise RuntimeError(
                    'contact-attitude pre-arm Vicon quaternion is invalid'
                )

    def activate(
            self, *, mode, experiment_run,
            vicon_orientation_forwarded):
        """Enable enqueueing only after the verified handle reaches runtime."""
        with self._lock:
            if self._closed or not self._accepting_callbacks:
                raise RuntimeError(
                    'contact-attitude pre-arm handle was already closed'
                )
            if not self._verified:
                raise RuntimeError(
                    'contact-attitude pre-arm readiness was not verified'
                )
            if self._active:
                raise RuntimeError(
                    'contact-attitude pre-arm handle was already activated'
                )
            if (
                mode != self.mode
                or int(experiment_run) != self.experiment_run
                or bool(vicon_orientation_forwarded)
                != self.vicon_orientation_forwarded
            ):
                raise RuntimeError(
                    'contact-attitude runtime does not match pre-arm protocol'
                )
            self._active = True
        return self.shadow

    def close(self):
        """Disable callbacks and retry only unsubscribers that failed."""
        with self._lock:
            if self._closed:
                return
            if self._close_in_progress:
                return
            self._close_in_progress = True
            self._accepting_callbacks = False
            self._active = False
            unsubscribers = tuple(reversed(self._unsubscribers))
        first_error = None
        failed = []
        for unsubscribe in unsubscribers:
            try:
                unsubscribe()
            except Exception as error:  # cleanup must attempt every callback
                failed.append(unsubscribe)
                if first_error is None:
                    first_error = error
        with self._lock:
            # Preserve original registration order so the next close() still
            # retries in reverse order, without repeating successful cleanup.
            self._unsubscribers = list(reversed(failed))
            self._closed = not failed
            self._close_in_progress = False
        if first_error is not None:
            raise first_error

    @property
    def verified(self):
        with self._lock:
            return self._verified

    @property
    def active(self):
        with self._lock:
            return self._active

    @property
    def closed(self):
        with self._lock:
            return self._closed
