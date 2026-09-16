"""One-shot, shadow-only Pi receive-time release transport.

The Pi monotonic timestamp is never represented as a Crazyflie timestamp.
The packet carries only the elapsed time since the first-unloaded CSV was
received. Firmware stamps its own packet-handler arrival; subtracting the
elapsed time is a delayed proxy, not a calibrated release epoch. Neither
endpoint grants estimator or HLC command authority.
"""

from __future__ import annotations

import math
import struct
import time

from cflib.crtp.crtpstack import CRTPPacket, CRTPPort


EVENT_TYPE = 14
EVENT_VERSION = 1
GENERIC_CHANNEL = 1
MAX_EVENT_AGE_US = 1_000_000
EVENT_STRUCT = struct.Struct("<BBHIII")
ACK_STRUCT = struct.Struct("<BBHIIII")


def encode_release_event(session_id, sequence, arduino_sample_ms,
                         pi_receive_monotonic_s, *, send_monotonic_ns):
    """Encode a frozen receive event; reject stale or invalid identities."""
    if (type(session_id) is not int or not 0 <= session_id <= 0xFFFFFFFF
            or type(sequence) is not int or not 0 <= sequence <= 0xFFFF
            or type(arduino_sample_ms) is not int
            or not 0 <= arduino_sample_ms <= 0xFFFFFFFF):
        raise ValueError("release sequence and Arduino sample ID invalid")
    if (isinstance(pi_receive_monotonic_s, bool)
            or not isinstance(pi_receive_monotonic_s, (float, int))
            or not math.isfinite(pi_receive_monotonic_s)
            or pi_receive_monotonic_s < 0
            or type(send_monotonic_ns) is not int):
        raise ValueError("Pi receive/send monotonic time invalid")
    event_ns = round(pi_receive_monotonic_s * 1_000_000_000)
    elapsed_ns = send_monotonic_ns - event_ns
    if elapsed_ns < 0 or elapsed_ns > MAX_EVENT_AGE_US * 1000:
        raise ValueError("release receive event is future-dated or stale")
    elapsed_us = elapsed_ns // 1000
    return EVENT_STRUCT.pack(
        EVENT_TYPE, EVENT_VERSION, sequence, session_id,
        arduino_sample_ms, elapsed_us
    ), elapsed_us


def send_release_event_diagnostic(
        cf, *, session_id, sequence, arduino_sample_ms,
        pi_receive_monotonic_s,
        monotonic_ns=time.monotonic_ns):
    """Send one diagnostic CRTP packet, without waiting in the control loop."""
    send_ns = monotonic_ns()
    payload, elapsed_us = encode_release_event(
        session_id, sequence, arduino_sample_ms, pi_receive_monotonic_s,
        send_monotonic_ns=send_ns,
    )
    packet = CRTPPacket()
    packet.set_header(CRTPPort.LOCALIZATION, GENERIC_CHANNEL)
    packet.data = payload
    cf.send_packet(packet)
    return {
        "session_id": session_id,
        "sequence": sequence,
        "release_event_arduino_time_ms": arduino_sample_ms,
        "pi_release_receive_monotonic_s": pi_receive_monotonic_s,
        "pi_send_start_monotonic_ns": send_ns,
        "pi_receive_to_send_elapsed_us": elapsed_us,
        "radio_delivery_confirmed": False,
        "release_epoch_calibrated": False,
        "command_authority": False,
    }


def parse_release_event_ack(packet, *, session_id, sequence,
                            arduino_sample_ms):
    """Read firmware evidence without interpreting its clock as Pi time."""
    if (packet.port != CRTPPort.LOCALIZATION
            or packet.channel != GENERIC_CHANNEL):
        return None
    payload = bytes(packet.data)
    if len(payload) != ACK_STRUCT.size:
        return None
    (kind, version, received_sequence, received_session, sample_id,
     elapsed_us, cf_receive_us) = (
        ACK_STRUCT.unpack(payload)
    )
    if (kind != EVENT_TYPE or version != EVENT_VERSION
            or received_sequence != sequence
            or received_session != session_id
            or sample_id != arduino_sample_ms
            or elapsed_us > MAX_EVENT_AGE_US):
        return None
    return {
        "session_id": received_session,
        "sequence": received_sequence,
        "release_event_arduino_time_ms": sample_id,
        "pi_receive_to_send_elapsed_us": elapsed_us,
        "firmware_packet_handler_receive_us_mod32": cf_receive_us,
        "delayed_release_proxy_us_mod32": (
            cf_receive_us - elapsed_us
        ) % (1 << 32),
        "radio_delivery_confirmed": True,
        "release_epoch_calibrated": False,
        "command_authority": False,
    }
