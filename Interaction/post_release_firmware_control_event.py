"""Opt-in Pi -> HLC release trigger for the paired hardware brake build.

The receive timestamp stays on the Pi. Firmware uses this packet as an event
boundary, then reads the estimator that has been running through contact.
Do not interpret the Arduino sample ID as a synchronized firmware timestamp.
Only the explicitly enabled Interaction firmware-auto-brake mode uses this
transport; the ordinary Interaction path remains unchanged.
"""

from __future__ import annotations

import math
import struct
import threading
import time

from cflib.crtp.crtpstack import CRTPPacket, CRTPPort


COMMAND_TYPE = 15
VERSION = 1
MAX_PI_EVENT_AGE_US = 250_000
PACKET = struct.Struct('<BBHIII')
VICON_POSITION_TYPE = 16
VICON_POSITION_PACKET = struct.Struct('<BIIfff')
HOLD_NOTICE_TYPE = 17
HOLD_ACK_TYPE = 18
HOLD_NOTICE_PACKET = struct.Struct('<BBHIffffI')
HOLD_ACK_PACKET = struct.Struct('<BBHI')
FIRMWARE_BRAKE_LOG_WARN_AGE_S = 0.35
FIRMWARE_BRAKE_LOG_FAIL_AGE_S = 0.80


def firmware_brake_abort_message(brake_log):
    """Describe the reported fault without guessing its underlying sensor."""
    reason = brake_log.get('hlCommander.pRelAbort')
    description = {
        1: 'trusted control state unavailable',
        2: 'unwind plan invalid',
        3: 'attitude/body-rate feedback unavailable',
        4: 'rate-aware unwind plan infeasible',
        11: 'Pi plan unavailable after bounded local return; not a stable hold',
    }.get(reason, 'reason not reported' if reason is None else 'unknown reason')
    return 'firmware brake aborted (stage 6; %s; reason=%s)' % (
        description, 'unavailable' if reason is None else reason)


def firmware_brake_log_health(receipt_time_s, *, now_s,
                              monitor_elapsed_s):
    """Distinguish a short status-log gap from unverified firmware control."""
    if receipt_time_s is None:
        age_s = monitor_elapsed_s
    elif (not math.isfinite(receipt_time_s) or
          not math.isfinite(now_s)):
        age_s = math.inf
    else:
        age_s = max(0.0, now_s - receipt_time_s)
    if age_s > FIRMWARE_BRAKE_LOG_FAIL_AGE_S:
        return 'expired', age_s
    if age_s > FIRMWARE_BRAKE_LOG_WARN_AGE_S:
        return 'delayed', age_s
    return ('waiting' if receipt_time_s is None else 'fresh'), age_s


def firmware_hold_status_confirmed(notice, brake_log, log_health):
    """A hold notice alone cannot substitute for a fresh stage-4 heartbeat."""
    return (notice is not None and log_health == 'fresh' and
            brake_log.get('hlCommander.pRelReady') == 1 and
            brake_log.get('hlCommander.pRelAutoSt') == 4)


class FirmwareBrakeMonitorError(RuntimeError):
    def __init__(self, message, *, code):
        super().__init__(message)
        self.code = code


class FirmwareBrakeMonitor:
    """Bound a single accepted release's heartbeat and terminal evidence.

    Receipt times identify packets; elapsed deadlines use the monotonic clock.
    An unchanged pre-release packet cannot prove completion of this release.
    Once received, a firmware abort remains a fault even if its packet ages.
    """

    def __init__(self, *, started_monotonic_s, baseline_receipt_time_s,
                 baseline_timeouts):
        self.started_s = started_monotonic_s
        self.baseline_receipt_s = baseline_receipt_time_s
        self.baseline_timeouts = baseline_timeouts
        self.last_receipt_s = None
        self.last_observed_s = None
        self.last_initial_age_s = 0.0
        self.missing_ready_since_s = None
        self.delayed_reported = False
        self.fault = None
        self.accepted_plan_end_s = None

    def track_accepted_plan(self, status):
        """Use confirmed FC acceptance, not a receipt-relative renewable grace."""
        if (status.get('phase') not in ('accepted', 'executing') or
                status.get('firmware_errno') != 0):
            return
        try:
            start = float(status['snapshot_received_s'])
            delay = float(status['start_delay_us']) / 1e6
            duration = float(status['duration_s'])
        except (KeyError, TypeError, ValueError):
            return
        end = start + delay + duration
        if (all(math.isfinite(v) for v in (start, delay, duration, end)) and
                0 <= delay <= 1 and 0 < duration <= 3 and
                self.started_s <= start and end <= self.started_s + 5):
            self.accepted_plan_end_s = end

    def _fail(self, code, message):
        self.fault = FirmwareBrakeMonitorError(message, code=code)
        raise self.fault

    def observe(self, brake_log, *, receipt_time_s, now_wall_s,
                now_monotonic_s, notice=None):
        if self.fault is not None:
            raise self.fault
        elapsed_s = now_monotonic_s - self.started_s
        post_release_packet = (receipt_time_s is not None and
                               receipt_time_s != self.baseline_receipt_s)
        new_packet = (post_release_packet and
                      receipt_time_s != self.last_receipt_s)
        if new_packet:
            self.last_receipt_s = receipt_time_s
            self.last_observed_s = now_monotonic_s
            _, self.last_initial_age_s = firmware_brake_log_health(
                receipt_time_s, now_s=now_wall_s,
                monitor_elapsed_s=elapsed_s)
        if self.last_observed_s is None:
            health, age_s = firmware_brake_log_health(
                None, now_s=now_wall_s, monitor_elapsed_s=elapsed_s)
        else:
            age_s = (self.last_initial_age_s +
                     now_monotonic_s - self.last_observed_s)
            health, _ = firmware_brake_log_health(
                0.0, now_s=age_s, monitor_elapsed_s=elapsed_s)

        # A known fault has precedence over telemetry freshness/readiness.
        if post_release_packet:
            if brake_log.get('hlCommander.pRelAutoSt') == 6:
                self._fail('firmware_abort', firmware_brake_abort_message(brake_log))
            timeouts = brake_log.get('hlCommander.pRelAutoTime')
            if (timeouts is not None and self.baseline_timeouts is not None and
                    timeouts != self.baseline_timeouts):
                self._fail('brake_timeout',
                           'firmware maximum-attitude brake timed out')

        if health == 'expired':
            self._fail('status_expired',
                       'firmware brake status log exceeded bounded grace '
                       '(age %.3f s)' % age_s)
        events = []
        if health == 'delayed' and not self.delayed_reported:
            self.delayed_reported = True
            events.append(('Firmware Brake Status Log Delayed', {'age_s': age_s}))
        elif health == 'fresh' and self.delayed_reported:
            self.delayed_reported = False
            events.append(('Firmware Brake Status Log Recovered', {'age_s': age_s}))

        if health == 'fresh' and post_release_packet:
            if brake_log.get('hlCommander.pRelReady') == 1:
                if self.missing_ready_since_s is not None:
                    events.append(('Firmware Brake Observer Recovered', {
                        'elapsed_s': now_monotonic_s - self.missing_ready_since_s}))
                self.missing_ready_since_s = None
            elif self.missing_ready_since_s is None:
                self.missing_ready_since_s = now_monotonic_s
                events.append(('Firmware Brake Observer Warning', {
                    'firmware_stage': brake_log.get('hlCommander.pRelAutoSt')}))
            if (brake_log.get('hlCommander.pRelAutoSt') == 0 and
                    elapsed_s > 0.35):
                self._fail('ownership_lost',
                           'firmware brake dropped ownership before hold')
        if (self.missing_ready_since_s is not None and
                now_monotonic_s - self.missing_ready_since_s > 0.30):
            bounded_curve = (health == 'fresh' and post_release_packet and
                             brake_log.get('hlCommander.pRelAutoSt') == 2 and
                             self.accepted_plan_end_s is not None and
                             now_monotonic_s <= self.accepted_plan_end_s + 0.30)
            if not bounded_curve:
                self._fail('readiness_lost',
                           'firmware control-state readiness unavailable beyond '
                           '0.30 s / confirmed bounded curve recovery deadline')

        return {
            'health': health,
            'age_s': age_s,
            'events': events,
            'hold_confirmed': (post_release_packet and
                               firmware_hold_status_confirmed(notice, brake_log, health)),
        }


def parse_post_release_hold_notice(packet, *, session_id, sequence):
    """Accept only this release's measured, firmware-established HLC hold."""
    if packet.port != CRTPPort.SETPOINT_HL or packet.channel != 1:
        return None
    data = bytes(packet.data)
    if len(data) != HOLD_NOTICE_PACKET.size:
        return None
    (kind, version, observed_sequence, observed_session, x, y, z,
     yaw_rad, hold_us) = HOLD_NOTICE_PACKET.unpack(data)
    if (kind != HOLD_NOTICE_TYPE or version != VERSION or
            observed_session != session_id or observed_sequence != sequence or
            not all(math.isfinite(value) for value in (x, y, z, yaw_rad))):
        return None
    return {
        'session_id': observed_session,
        'sequence': observed_sequence,
        'hold_position_m': [x, y, z],
        'hold_yaw_rad': yaw_rad,
        'firmware_hold_us_mod32': hold_us,
    }


class FirmwareHoldNotification:
    """Listen before release; acknowledge a matching terminal hold once seen."""

    def __init__(self, cf, *, session_id, sequence):
        self.cf = cf
        self.session_id = session_id
        self.sequence = sequence
        self.received = threading.Event()
        self.notice = None
        self._callback = self._on_packet

    def __enter__(self):
        self.cf.add_port_callback(CRTPPort.SETPOINT_HL, self._callback)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.cf.remove_port_callback(CRTPPort.SETPOINT_HL, self._callback)

    def _on_packet(self, packet):
        notice = parse_post_release_hold_notice(
            packet, session_id=self.session_id, sequence=self.sequence,
        )
        if notice is not None and not self.received.is_set():
            self.notice = notice
            self.received.set()

    def wait(self, timeout_s):
        return self.notice if self.received.wait(timeout_s) else None

    def acknowledge(self):
        if self.notice is None:
            raise RuntimeError('cannot acknowledge an unobserved hold')
        packet = CRTPPacket()
        packet.set_header(CRTPPort.SETPOINT_HL, 0)
        packet.data = HOLD_ACK_PACKET.pack(
            HOLD_ACK_TYPE, VERSION, self.sequence, self.session_id,
        )
        self.cf.send_packet(packet)


def send_vicon_position_mirror(cf, position_m, *,
                               pi_receive_monotonic_s,
                               monotonic_s=time.monotonic):
    """Mirror a received Vicon position without altering ordinary extpos.

    The Pi clock supplies only a local age and a modulo-32 receive epoch.
    Firmware derives inter-frame intervals from epochs it actually received;
    it never interprets the Pi timestamp as firmware or camera uptime.
    """
    if (len(position_m) != 3 or not all(
            isinstance(value, (int, float)) and
            math.isfinite(value) for value in position_m)):
        raise ValueError('Vicon position must be three finite coordinates')
    if not (isinstance(pi_receive_monotonic_s, (int, float)) and
            math.isfinite(pi_receive_monotonic_s) and
            pi_receive_monotonic_s >= 0):
        raise ValueError('Pi Vicon receive time must be monotonic')
    receive_us_mod32 = round(pi_receive_monotonic_s * 1_000_000) & 0xFFFFFFFF
    age_us = round((monotonic_s() - pi_receive_monotonic_s) * 1_000_000)
    if not 0 <= age_us <= 10_000:
        raise ValueError('Vicon position mirror has stale timing')
    packet = CRTPPacket()
    packet.set_header(CRTPPort.LOCALIZATION, 1)
    packet.data = VICON_POSITION_PACKET.pack(
        VICON_POSITION_TYPE, receive_us_mod32, age_us, *position_m)
    cf.send_packet(packet)
    return {'pi_receive_us_mod32': receive_us_mod32,
            'pi_receive_to_send_us': age_us}


def encode_pi_release_command(*, session_id, sequence, arduino_sample_ms,
                              pi_receive_monotonic_ns, send_monotonic_ns):
    for name, value, limit in (
        ('session_id', session_id, 0xFFFFFFFF),
        ('sequence', sequence, 0xFFFF),
        ('arduino_sample_ms', arduino_sample_ms, 0xFFFFFFFF),
    ):
        if type(value) is not int or not 0 <= value <= limit:
            raise ValueError(f'{name} is invalid')
    if (type(pi_receive_monotonic_ns) is not int or
            type(send_monotonic_ns) is not int):
        raise ValueError('Pi receive/send timestamps must be monotonic ns')
    elapsed_ns = send_monotonic_ns - pi_receive_monotonic_ns
    if not 0 <= elapsed_ns <= MAX_PI_EVENT_AGE_US * 1000:
        raise ValueError('Pi release receive event is future-dated or stale')
    elapsed_us = elapsed_ns // 1000
    return PACKET.pack(COMMAND_TYPE, VERSION, sequence, session_id,
                       arduino_sample_ms, elapsed_us), elapsed_us


def send_pi_release_command_once(
        cf, *, session_id, sequence, arduino_sample_ms,
        pi_receive_monotonic_ns, firmware_auto_brake_armed=False,
        monotonic_ns=time.monotonic_ns):
    """Send once only after explicit firmware capability/arming validation.

    The HLC acknowledgement is separate; losing it must not cause automatic
    retransmission. No ordinary Interaction mode calls this function.
    """
    if firmware_auto_brake_armed is not True:
        raise ValueError('firmware auto brake must be explicitly armed')
    send_ns = monotonic_ns()
    payload, elapsed_us = encode_pi_release_command(
        session_id=session_id, sequence=sequence,
        arduino_sample_ms=arduino_sample_ms,
        pi_receive_monotonic_ns=pi_receive_monotonic_ns,
        send_monotonic_ns=send_ns,
    )
    packet = CRTPPacket()
    packet.set_header(CRTPPort.SETPOINT_HL, 0)
    packet.data = payload
    cf.send_packet(packet)
    send_return_ns = monotonic_ns()
    return {
        'session_id': session_id,
        'sequence': sequence,
        'release_event_arduino_time_ms': arduino_sample_ms,
        'request_payload_hex': payload.hex(),
        'pi_release_receive_monotonic_ns': pi_receive_monotonic_ns,
        'pi_send_start_monotonic_ns': send_ns,
        'pi_send_return_monotonic_ns': send_return_ns,
        'pi_receive_to_send_elapsed_us': elapsed_us,
        'radio_delivery_confirmed': False,
        'firmware_command_started': False,
    }


def parse_pi_release_ack(packet, *, request_payload_hex,
                         ack_receive_monotonic_ns):
    """Match the whole echoed request; ACK means queued, not braking begun."""
    if (packet.port != CRTPPort.SETPOINT_HL or packet.channel != 0 or
            type(ack_receive_monotonic_ns) is not int or
            ack_receive_monotonic_ns < 0):
        return None
    try:
        request = bytes.fromhex(request_payload_hex)
    except (TypeError, ValueError):
        return None
    data = bytes(packet.data)
    if (len(request) != PACKET.size or
            len(data) != PACKET.size + 1 or data[:-1] != request):
        return None
    return {
        'ack_errno': data[-1],
        'ack_receive_monotonic_ns': ack_receive_monotonic_ns,
        'radio_delivery_confirmed': True,
        'event_queued_by_firmware': data[-1] == 0,
        'firmware_command_started': False,
    }


def handoff_pi_release_to_firmware(
        cf, *, session_id, sequence, arduino_sample_ms,
        pi_receive_monotonic_ns, firmware_auto_brake_armed=False,
        ack_timeout_s=0.15, monotonic_ns=time.monotonic_ns):
    """Confirm a firmware-owned LL-to-HLC transfer using the matching ACK.

    A timeout is ambiguous: this function never retries the release event or
    sends a new position goal. Firmware claims priority before replying; the
    caller must use its existing landing path if the reply is lost or rejected.
    The ACK proves the event was accepted, not that braking or terminal hold
    has completed.
    """
    if firmware_auto_brake_armed is not True:
        raise ValueError('firmware auto brake must be explicitly armed')
    if not 0 < ack_timeout_s <= 0.15:
        raise ValueError('ACK timeout must be within 0.15 s')
    received = threading.Event()
    matched = {}
    request = None

    def on_packet(packet):
        if request is None or received.is_set():
            return
        result = parse_pi_release_ack(
            packet, request_payload_hex=request,
            ack_receive_monotonic_ns=monotonic_ns(),
        )
        if result is not None:
            matched.update(result)
            received.set()

    cf.add_port_callback(CRTPPort.SETPOINT_HL, on_packet)
    try:
        # Register before sending so an immediate Bolt reply cannot be lost.
        send_ns = monotonic_ns()
        payload, elapsed_us = encode_pi_release_command(
            session_id=session_id, sequence=sequence,
            arduino_sample_ms=arduino_sample_ms,
            pi_receive_monotonic_ns=pi_receive_monotonic_ns,
            send_monotonic_ns=send_ns,
        )
        request = payload.hex()
        packet = CRTPPacket()
        packet.set_header(CRTPPort.SETPOINT_HL, 0)
        packet.data = payload
        cf.send_packet(packet)
        if not received.wait(ack_timeout_s):
            raise RuntimeError('firmware release event ACK timed out; LL ownership retained')
        if not matched['event_queued_by_firmware']:
            raise RuntimeError(
                f"firmware rejected release event (errno={matched['ack_errno']})"
            )
        return {
            'session_id': session_id,
            'sequence': sequence,
            'release_event_arduino_time_ms': arduino_sample_ms,
            'pi_release_receive_monotonic_ns': pi_receive_monotonic_ns,
            'pi_send_start_monotonic_ns': send_ns,
            'pi_receive_to_send_elapsed_us': elapsed_us,
            **matched,
            'low_level_priority_released': True,
            'firmware_priority_claimed': True,
        }
    finally:
        cf.remove_port_callback(CRTPPort.SETPOINT_HL, on_packet)
