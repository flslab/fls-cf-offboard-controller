"""Shadow-only fixed-delay release packet v2, with no per-event Pi time."""

from __future__ import annotations

import hashlib
import json
import math
import struct
from pathlib import Path

from cflib.crtp.crtpstack import CRTPPacket, CRTPPort

from Interaction.post_release_event_diagnostic import GENERIC_CHANNEL


EVENT_TYPE = 14
EVENT_VERSION = 2
REQUEST = struct.Struct("<BBHII")
REPLY = struct.Struct("<BBHIIII")


def calibrated_fixed_delay_us(document):
    if document.get("schema") != "bolt_clock_rate_prop_off_batch_v1":
        raise ValueError("clock-rate calibration batch invalid")
    clock = document["clock_fit"]
    release = document["release_fit"]
    fixed = document["firmware_delay_shadow"]
    if (not clock["holdout_compatible"]
            or not release["holdout_covered"]
            or fixed["command_authority"]):
        raise ValueError("calibration is not valid shadow evidence")
    value = fixed["fixed_delay_firmware_ms"]
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value)):
        raise ValueError("fixed delay invalid")
    microseconds = round(value * 1000)
    if not 1 <= microseconds <= 1000000:
        raise ValueError("fixed delay outside diagnostic range")
    return microseconds


def load_fixed_event_calibration(path):
    calibration_bytes = Path(path).read_bytes()
    document = json.loads(calibration_bytes)
    return {
        "fixed_delay_firmware_us": calibrated_fixed_delay_us(document),
        "calibration_sha256": hashlib.sha256(calibration_bytes).hexdigest(),
    }


def encode_fixed_release_event(session_id, sequence, arduino_sample_ms):
    if (type(session_id) is not int or not 0 <= session_id <= 0xFFFFFFFF
            or type(sequence) is not int or not 0 <= sequence <= 0xFFFF
            or type(arduino_sample_ms) is not int
            or not 0 <= arduino_sample_ms <= 0xFFFFFFFF):
        raise ValueError("release event identity invalid")
    return REQUEST.pack(
        EVENT_TYPE, EVENT_VERSION, sequence, session_id, arduino_sample_ms,
    )


def send_fixed_release_event_diagnostic(
        cf, *, session_id, sequence, arduino_sample_ms, calibration):
    fixed_delay_us = calibration["fixed_delay_firmware_us"]
    if type(fixed_delay_us) is not int or not 1 <= fixed_delay_us <= 1000000:
        raise ValueError("fixed-delay calibration invalid")
    payload = encode_fixed_release_event(
        session_id, sequence, arduino_sample_ms
    )
    packet = CRTPPacket()
    packet.set_header(CRTPPort.LOCALIZATION, GENERIC_CHANNEL)
    packet.data = payload
    cf.send_packet(packet)
    return {
        "session_id": session_id,
        "sequence": sequence,
        "release_event_arduino_time_ms": arduino_sample_ms,
        "firmware_fixed_delay_us": fixed_delay_us,
        "calibration_sha256": calibration["calibration_sha256"],
        "payload_size_bytes": len(payload),
        "per_event_elapsed_sent": False,
        "pi_timestamp_sent": False,
        "radio_delivery_confirmed": False,
        "command_authority": False,
    }


def parse_fixed_release_ack(packet, *, session_id, sequence,
                            arduino_sample_ms, fixed_delay_us):
    if (packet.port != CRTPPort.LOCALIZATION
            or packet.channel != GENERIC_CHANNEL
            or len(packet.data) != REPLY.size):
        return None
    (kind, version, actual_sequence, actual_session, actual_sample,
     echoed_delay_us, receive_us) = REPLY.unpack(bytes(packet.data))
    if (kind != EVENT_TYPE or version != EVENT_VERSION
            or actual_sequence != sequence
            or actual_session != session_id
            or actual_sample != arduino_sample_ms
            or echoed_delay_us != fixed_delay_us):
        return None
    return {
        "session_id": actual_session,
        "sequence": actual_sequence,
        "release_event_arduino_time_ms": actual_sample,
        "firmware_fixed_delay_us": echoed_delay_us,
        "firmware_packet_handler_receive_us_mod32": receive_us,
        "firmware_release_receive_proxy_us_mod32": (
            receive_us - echoed_delay_us
        ) % (1 << 32),
        "radio_delivery_confirmed": True,
        "release_epoch_calibrated": False,
        "command_authority": False,
    }
