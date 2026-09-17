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
        cf, low_level, *, session_id, sequence, arduino_sample_ms,
        pi_receive_monotonic_ns, firmware_auto_brake_armed=False,
        ack_timeout_s=0.15, monotonic_ns=time.monotonic_ns):
    """Transfer LL priority only after the matching firmware event ACK.

    A timeout is ambiguous: this function never retries the release event or
    sends a new position goal. The caller must use its existing landing path.
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
        low_level.send_notify_setpoint_stop()
        return {
            'session_id': session_id,
            'sequence': sequence,
            'release_event_arduino_time_ms': arduino_sample_ms,
            'pi_release_receive_monotonic_ns': pi_receive_monotonic_ns,
            'pi_send_start_monotonic_ns': send_ns,
            'pi_receive_to_send_elapsed_us': elapsed_us,
            **matched,
            'low_level_priority_released': True,
        }
    finally:
        cf.remove_port_callback(CRTPPort.SETPOINT_HL, on_packet)
