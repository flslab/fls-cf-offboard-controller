"""Read-only Bolt CRTP clock brackets; never arms or sends setpoints.

Requires the opt-in post-release TIME_PROBE diagnostic firmware. Run with no
other client using the radio. Output gives causal clock-offset intervals, not
a release timestamp or HLC authorization.
"""

from __future__ import annotations

import argparse
import json
import statistics
import struct
import threading
import time

import cflib.crtp
from cflib.crtp.crtpstack import CRTPPacket, CRTPPort
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


PROBE_TYPE = 13
GENERIC_CHANNEL = 1
MICROS_MODULUS = 1 << 32


def parse_probe_reply(packet, expected_sequence):
    if packet.port != CRTPPort.LOCALIZATION or packet.channel != GENERIC_CHANNEL:
        return None
    payload = bytes(packet.data)
    if len(payload) != 11:
        return None
    kind, sequence, receive_us, enqueue_us = struct.unpack("<BHII", payload)
    if kind != PROBE_TYPE or sequence != expected_sequence:
        return None
    processing_us = (enqueue_us - receive_us) % MICROS_MODULUS
    if processing_us > 1000000:
        return None
    return receive_us, enqueue_us, processing_us


def causal_offset_bounds_us(send_start_ns, reply_end_ns,
                            firmware_receive_us, firmware_enqueue_us):
    """Bound CF-minus-host clock offset without one-way delay symmetry."""
    if (
        reply_end_ns <= send_start_ns
        or firmware_enqueue_us < firmware_receive_us
    ):
        raise ValueError("probe times are out of order")
    lower = firmware_enqueue_us - reply_end_ns / 1000.0
    upper = firmware_receive_us - send_start_ns / 1000.0
    if lower > upper:
        raise ValueError("probe times violate causality")
    return lower, upper


def collect_probes(uri, *, probes=32, timeout_s=0.5, interval_s=0.05):
    if not uri or not 2 <= probes <= 1000 or timeout_s <= 0.0:
        raise ValueError("uri, 2..1000 probes, and positive timeout required")
    if interval_s < 0.0:
        raise ValueError("interval must be nonnegative")
    cflib.crtp.init_drivers()
    rows = []
    condition = threading.Condition()
    pending = {"sequence": None, "reply": None}

    def on_packet(packet):
        received_ns = time.monotonic_ns()
        with condition:
            sequence = pending["sequence"]
            if sequence is None:
                return
            parsed = parse_probe_reply(packet, sequence)
            if parsed is not None:
                pending["reply"] = (received_ns, parsed)
                condition.notify_all()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache=None)) as scf:
        cf = scf.cf
        cf.add_header_callback(
            on_packet, CRTPPort.LOCALIZATION, GENERIC_CHANNEL
        )
        try:
            last_raw_receive = None
            unwrapped_receive = None
            for sequence in range(probes):
                packet = CRTPPacket()
                packet.set_header(CRTPPort.LOCALIZATION, GENERIC_CHANNEL)
                packet.data = struct.pack("<BH", PROBE_TYPE, sequence)
                with condition:
                    pending["sequence"] = sequence
                    pending["reply"] = None
                send_start_ns = time.monotonic_ns()
                cf.send_packet(packet)
                deadline = time.monotonic() + timeout_s
                with condition:
                    while pending["reply"] is None:
                        remaining = deadline - time.monotonic()
                        if remaining <= 0.0:
                            break
                        condition.wait(remaining)
                    reply = pending["reply"]
                    pending["sequence"] = None
                if reply is None:
                    raise RuntimeError(
                        f"no time-probe reply {sequence}; check diagnostic "
                        "firmware and radio link"
                    )
                reply_end_ns, (raw_receive, raw_enqueue, processing_us) = reply
                if last_raw_receive is None:
                    unwrapped_receive = raw_receive
                else:
                    advance_us = (raw_receive - last_raw_receive) % MICROS_MODULUS
                    if advance_us == 0 or advance_us >= MICROS_MODULUS // 2:
                        raise RuntimeError("Bolt clock reset or probe order invalid")
                    unwrapped_receive += advance_us
                last_raw_receive = raw_receive
                unwrapped_enqueue = unwrapped_receive + processing_us
                # Causality alone gives b = CF_time - host_time in this
                # interval. No symmetric radio-latency assumption is used.
                lower_offset_us, upper_offset_us = causal_offset_bounds_us(
                    send_start_ns, reply_end_ns,
                    unwrapped_receive, unwrapped_enqueue,
                )
                rows.append({
                    "sequence": sequence,
                    "host_send_start_monotonic_ns": send_start_ns,
                    "bolt_receive_us_mod32": raw_receive,
                    "bolt_response_enqueue_us_mod32": raw_enqueue,
                    "host_reply_end_monotonic_ns": reply_end_ns,
                    "firmware_processing_us": processing_us,
                    "offset_lower_us": lower_offset_us,
                    "offset_upper_us": upper_offset_us,
                    "causal_bracket_width_ms": (
                        upper_offset_us - lower_offset_us
                    ) / 1000.0,
                })
                if interval_s:
                    time.sleep(interval_s)
        finally:
            cf.remove_header_callback(
                on_packet, CRTPPort.LOCALIZATION, GENERIC_CHANNEL
            )
    widths = [row["causal_bracket_width_ms"] for row in rows]
    return {
        "schema": "bolt_radio_clock_brackets_v1",
        "uri": uri,
        "probes": rows,
        "minimum_bracket_width_ms": min(widths),
        "median_bracket_width_ms": statistics.median(widths),
        "maximum_bracket_width_ms": max(widths),
        "release_epoch_calibrated": False,
        "position_capture_time_calibrated": False,
        "command_authority": False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uri", required=True)
    parser.add_argument("--probes", type=int, default=32)
    parser.add_argument("--timeout-s", type=float, default=0.5)
    parser.add_argument("--interval-s", type=float, default=0.05)
    args = parser.parse_args()
    print(json.dumps(collect_probes(
        args.uri, probes=args.probes,
        timeout_s=args.timeout_s, interval_s=args.interval_s,
    ), indent=2))


if __name__ == "__main__":
    main()
