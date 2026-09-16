"""Prop-off fixed-delay release-event v2 probe; never arms or sends setpoints.

Requires a separately built/flashed opt-in diagnostic firmware. The request
contains event identity only: no Pi timestamp and no per-event elapsed time.
Firmware echoes its fixed delay and packet-handler receipt timestamp. The
result is shadow evidence, not an EKF release latch or HLC authorization.
"""

from __future__ import annotations

import argparse
import json
import secrets
import threading
import time
from pathlib import Path

import cflib.crtp
import serial
from cflib.crtp.crtpstack import CRTPPort
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from Interaction.post_release_event_diagnostic import GENERIC_CHANNEL
from Interaction.post_release_fixed_event_diagnostic import (
    load_fixed_event_calibration,
    parse_fixed_release_ack,
    send_fixed_release_event_diagnostic,
)
from Interaction.probe_post_release_event import collect_manual_release


def probe(uri, port, calibration_path, *, timeout_s=60.0,
          ack_timeout_s=1.0, contact_force_n=0.18,
          unloaded_force_n=0.17, force_drop_n=0.04,
          decrease_rate_n_s=0.05, unloaded_dwell_s=0.05):
    if not uri or not port or timeout_s <= 0 or ack_timeout_s <= 0:
        raise ValueError("URI, port, and positive timeouts required")
    calibration = load_fixed_event_calibration(calibration_path)
    fixed_delay_us = calibration["fixed_delay_firmware_us"]
    session_id = secrets.randbits(32)
    sequence = 0
    received = {}
    ack_event = threading.Event()

    def on_packet(packet):
        parsed = parse_fixed_release_ack(
            packet, session_id=session_id, sequence=sequence,
            arduino_sample_ms=received.get("sample_id"),
            fixed_delay_us=fixed_delay_us,
        )
        if parsed is not None:
            received["ack"] = parsed
            received["ack_host_monotonic_ns"] = time.monotonic_ns()
            ack_event.set()

    cflib.crtp.init_drivers()
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache=None)) as scf:
        cf = scf.cf
        cf.add_header_callback(
            on_packet, CRTPPort.LOCALIZATION, GENERIC_CHANNEL
        )
        try:
            with serial.Serial(port, 115200, timeout=0.1) as connection:
                connection.reset_input_buffer()
                decision = collect_manual_release(
                    connection, timeout_s=timeout_s,
                    contact_force_n=contact_force_n,
                    unloaded_force_n=unloaded_force_n,
                    force_drop_n=force_drop_n,
                    decrease_rate_n_s=decrease_rate_n_s,
                    unloaded_dwell_s=unloaded_dwell_s,
                )
            received["sample_id"] = decision.unloaded_started_sample_id
            send_ns = time.monotonic_ns()
            sent = send_fixed_release_event_diagnostic(
                cf, session_id=session_id, sequence=sequence,
                arduino_sample_ms=received["sample_id"],
                calibration=calibration,
            )
            if not ack_event.wait(ack_timeout_s):
                raise TimeoutError(
                    "no matching fixed-delay v2 echo; no control was sent"
                )
            return {
                "schema": "post_release_fixed_event_prop_off_v2",
                "calibration_sha256": calibration["calibration_sha256"],
                "sent": sent,
                "ack": received["ack"],
                "host_radio_roundtrip_ms": (
                    received["ack_host_monotonic_ns"] - send_ns
                ) / 1e6,
                "force_event_time_basis": "pi_complete_uart_line_receive_monotonic",
                "physical_capture_time_calibrated": False,
                "command_authority": False,
            }
        finally:
            cf.remove_header_callback(
                on_packet, CRTPPort.LOCALIZATION, GENERIC_CHANNEL
            )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uri", required=True)
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--calibration-batch", type=Path, required=True)
    parser.add_argument("--timeout-s", type=float, default=60.0)
    args = parser.parse_args()
    print(json.dumps(probe(
        args.uri, args.port, args.calibration_batch,
        timeout_s=args.timeout_s,
    ), indent=2))


if __name__ == "__main__":
    main()
