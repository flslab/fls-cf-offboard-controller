"""Prop-off manual press/release probe; sends no flight setpoints.

Run alone with the normal Pi force reader and flight controller stopped.
Press the spring above the contact threshold, then fully release it. The
first-unloaded Pi UART receive time is frozen until dwell confirmation; one
shadow-only radio event is sent and its diagnostic firmware echo is checked.
This probe never arms, takes off, changes parameters, or grants HLC authority.
"""

from __future__ import annotations

import argparse
import json
import secrets
import sys
import threading
import time

import cflib.crtp
import serial
from cflib.crtp.crtpstack import CRTPPort
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

try:
    from Interaction.potentiometer_force_sensor import (
        PotentiometerReleaseDetector,
        parse_potentiometer_line,
    )
    from Interaction.post_release_event_diagnostic import (
        GENERIC_CHANNEL,
        parse_release_event_ack,
        send_release_event_diagnostic,
    )
except ModuleNotFoundError as error:
    if error.name != "Interaction":
        raise
    from potentiometer_force_sensor import (
        PotentiometerReleaseDetector,
        parse_potentiometer_line,
    )
    from post_release_event_diagnostic import (
        GENERIC_CHANNEL,
        parse_release_event_ack,
        send_release_event_diagnostic,
    )


def collect_manual_release(connection, *, timeout_s, contact_force_n,
                           unloaded_force_n, force_drop_n,
                           decrease_rate_n_s, unloaded_dwell_s):
    """Require an unloaded baseline, deliberate press, then confirmed release."""
    detector = PotentiometerReleaseDetector(
        force_drop_n=force_drop_n,
        decrease_rate_n_s=decrease_rate_n_s,
        unloaded_force_n=unloaded_force_n,
        unloaded_dwell_s=unloaded_dwell_s,
    )
    deadline = time.monotonic() + timeout_s
    baseline_count = 0
    loaded_since = None
    while time.monotonic() < deadline:
        line = connection.readline()
        receive_s = time.monotonic()
        sample = parse_potentiometer_line(
            line, host_monotonic_time=receive_s
        )
        if sample is None:
            continue
        force = sample.force_n
        if not detector.armed:
            if baseline_count < 3:
                baseline_count = baseline_count + 1 if force <= unloaded_force_n else 0
                continue
            if force < contact_force_n:
                loaded_since = None
                continue
            if loaded_since is None:
                loaded_since = receive_s
            if receive_s - loaded_since >= 0.03:
                detector.arm(force, receive_s)
            continue
        decision = detector.update(
            force, receive_s, sample_id=sample.arduino_time_ms
        )
        if decision.released:
            if (decision.unloaded_started_at_s is None
                    or decision.unloaded_started_sample_id is None):
                raise RuntimeError("confirmed release lacks first-unloaded identity")
            return decision
    raise TimeoutError("manual baseline, press, and release not completed")


def probe(uri, port, *, timeout_s=60.0, ack_timeout_s=1.0,
          contact_force_n=0.18, unloaded_force_n=0.17,
          force_drop_n=0.04, decrease_rate_n_s=0.05,
          unloaded_dwell_s=0.05):
    if not uri or not port or timeout_s <= 0 or ack_timeout_s <= 0:
        raise ValueError("URI, port, and positive timeouts required")
    session_id = secrets.randbits(32)
    sequence = 0
    received = {}
    ack_event = threading.Event()

    def on_packet(packet):
        parsed = parse_release_event_ack(
            packet, session_id=session_id, sequence=sequence,
            arduino_sample_ms=received.get("sample_id"),
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
            sent = send_release_event_diagnostic(
                cf, session_id=session_id, sequence=sequence,
                arduino_sample_ms=decision.unloaded_started_sample_id,
                pi_receive_monotonic_s=decision.unloaded_started_at_s,
            )
            if not ack_event.wait(ack_timeout_s):
                raise TimeoutError(
                    "no matching Bolt diagnostic echo; no control was sent"
                )
            return {
                "schema": "post_release_receive_event_prop_off_v1",
                "sent": sent,
                "ack": received["ack"],
                "host_radio_roundtrip_ms": (
                    received["ack_host_monotonic_ns"]
                    - sent["pi_send_start_monotonic_ns"]
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
    parser.add_argument("--timeout-s", type=float, default=60.0)
    parser.add_argument("--ack-timeout-s", type=float, default=1.0)
    parser.add_argument("--contact-force-n", type=float, default=0.18)
    parser.add_argument("--unloaded-force-n", type=float, default=0.17)
    parser.add_argument("--force-drop-n", type=float, default=0.04)
    parser.add_argument("--decrease-rate-n-s", type=float, default=0.05)
    parser.add_argument("--unloaded-dwell-s", type=float, default=0.05)
    parser.add_argument("--repeats", type=int, default=1,
                        help="manual prop-off release cycles; default preserves one-shot output")
    parser.add_argument("--maximum-uncertainty-ms", type=float)
    args = parser.parse_args()
    if not 1 <= args.repeats <= 100:
        parser.error("--repeats must be between 1 and 100")
    records = []
    for index in range(args.repeats):
        if args.repeats > 1:
            print(f"Prop-off manual press/release {index + 1}/{args.repeats}",
                  file=sys.stderr, flush=True)
        try:
            records.append(probe(
                args.uri, args.port, timeout_s=args.timeout_s,
                ack_timeout_s=args.ack_timeout_s,
                contact_force_n=args.contact_force_n,
                unloaded_force_n=args.unloaded_force_n,
                force_drop_n=args.force_drop_n,
                decrease_rate_n_s=args.decrease_rate_n_s,
                unloaded_dwell_s=args.unloaded_dwell_s,
            ))
        except (RuntimeError, TimeoutError, OSError) as error:
            if args.repeats == 1:
                raise
            print(json.dumps({
                "schema": "post_release_prop_off_calibration_batch_v1",
                "records": records,
                "calibration": None,
                "incomplete_error": str(error),
                "command_authority": False,
            }, indent=2))
            raise SystemExit(1) from error
    if args.repeats == 1:
        result = records[0]
    else:
        from Interaction.calibrate_release_transport_delay import (
            fit_release_transport_delay,
        )
        result = {
            "schema": "post_release_prop_off_calibration_batch_v1",
            "records": records,
            "calibration": fit_release_transport_delay(
                records, maximum_uncertainty_ms=args.maximum_uncertainty_ms,
            ) if len(records) >= 20 else None,
            "command_authority": False,
        }
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
