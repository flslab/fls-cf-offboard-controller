"""Prop-off serial diagnostics for the Arduino potentiometer stream.

The default two-way mode measures causal host-send/Arduino-receive/host-reply
brackets. --receive-only checks the existing one-way CSV cadence without
estimating a clock offset or a delay bound. Neither mode synchronizes the
Crazyflie clock or authorizes post-release control. Run with the normal
force-sensor reader stopped so one process owns the serial port.
"""

from __future__ import annotations

import argparse
import json
import statistics
import time

import serial

try:
    from Interaction.potentiometer_force_sensor import parse_potentiometer_line
except ModuleNotFoundError as error:
    if error.name != "Interaction":
        raise
    # Also support direct execution from the Interaction/ directory.
    from potentiometer_force_sensor import parse_potentiometer_line


def parse_sync_reply(line, expected_sequence):
    """Return an Arduino micros stamp only for this probe's exact reply."""
    try:
        text = line.decode("ascii") if isinstance(line, bytes) else str(line)
        parts = text.strip().split(",")
        if len(parts) != 3 or parts[0] != "#SYNC":
            return None
        if any(not part.isascii() or not part.isdecimal() for part in parts[1:]):
            return None
        sequence = int(parts[1])
        stamp = int(parts[2])
        if sequence != expected_sequence or not 0 <= stamp < 1 << 32:
            return None
        return stamp
    except (UnicodeError, ValueError):
        return None


def collect_probes(port, *, baud=115200, probes=32,
                   timeout_s=0.5, interval_s=0.05, warmup_s=2.0):
    if not port or probes < 2 or probes > 1000 or timeout_s <= 0.0:
        raise ValueError("port, 2..1000 probes, and positive timeout required")
    if interval_s < 0.0 or warmup_s < 0.0:
        raise ValueError("interval and warmup must be nonnegative")
    rows = []
    with serial.Serial(port, baudrate=baud, timeout=0.02,
                       write_timeout=timeout_s) as connection:
        time.sleep(warmup_s)  # Classic Nano resets when serial opens.
        connection.reset_input_buffer()
        for sequence in range(probes):
            request = f"T,{sequence}\n".encode("ascii")
            host_send_start_ns = time.monotonic_ns()
            connection.write(request)
            connection.flush()
            deadline_ns = host_send_start_ns + int(timeout_s * 1e9)
            received = None
            while time.monotonic_ns() < deadline_ns:
                response = connection.readline()
                host_reply_end_ns = time.monotonic_ns()
                stamp = parse_sync_reply(response, sequence)
                if stamp is not None:
                    received = {
                        "sequence": sequence,
                        "host_send_start_monotonic_ns": host_send_start_ns,
                        "arduino_request_complete_us_mod32": stamp,
                        "host_reply_end_monotonic_ns": host_reply_end_ns,
                        "causal_bracket_width_ms": (
                            host_reply_end_ns - host_send_start_ns
                        ) / 1e6,
                    }
                    break
            if received is None:
                raise RuntimeError(
                    f"no #SYNC reply for probe {sequence}; check Pi TX/RX "
                    "wiring and the uploaded Arduino sketch"
                )
            rows.append(received)
            if interval_s:
                time.sleep(interval_s)
    for previous, current in zip(rows, rows[1:]):
        delta_us = (
            current["arduino_request_complete_us_mod32"]
            - previous["arduino_request_complete_us_mod32"]
        ) % (1 << 32)
        if delta_us == 0 or delta_us >= 1 << 31:
            raise RuntimeError("Arduino clock reset or nonmonotonic probe")
    widths = [row["causal_bracket_width_ms"] for row in rows]
    return {
        "schema": "arduino_serial_clock_brackets_v1",
        "port": port,
        "baud": baud,
        "probes": rows,
        "minimum_bracket_width_ms": min(widths),
        "median_bracket_width_ms": statistics.median(widths),
        "maximum_bracket_width_ms": max(widths),
        "crazyflie_clock_calibrated": False,
        "force_sample_capture_time_calibrated": False,
        "command_authority": False,
    }


def collect_receive_only(port, *, baud=115200, samples=100,
                         timeout_s=0.5, warmup_s=0.0):
    """Check the existing one-way force stream without claiming clock sync."""
    if not port or not 2 <= samples <= 10000 or timeout_s <= 0.0:
        raise ValueError("port, 2..10000 samples, and positive timeout required")
    if warmup_s < 0.0:
        raise ValueError("warmup must be nonnegative")
    rows = []
    ignored_lines = 0
    with serial.Serial(port, baudrate=baud, timeout=timeout_s) as connection:
        if warmup_s:
            time.sleep(warmup_s)
        connection.reset_input_buffer()
        deadline = time.monotonic() + max(5.0, samples * 0.1)
        while len(rows) < samples and time.monotonic() < deadline:
            line = connection.readline()
            received_ns = time.monotonic_ns()
            sample = parse_potentiometer_line(line)
            if sample is None:
                ignored_lines += 1
                continue
            rows.append({
                "arduino_sample_ms_mod32": sample.arduino_time_ms,
                "host_receive_monotonic_ns": received_ns,
            })
    if len(rows) != samples:
        raise RuntimeError(
            f"only {len(rows)}/{samples} force samples received from {port}"
        )
    arduino_steps = []
    host_steps = []
    for previous, current in zip(rows, rows[1:]):
        delta_ms = (
            current["arduino_sample_ms_mod32"]
            - previous["arduino_sample_ms_mod32"]
        ) % (1 << 32)
        if delta_ms == 0 or delta_ms >= 1 << 31:
            raise RuntimeError("Arduino sample clock reset or nonmonotonic")
        arduino_steps.append(delta_ms)
        host_steps.append((
            current["host_receive_monotonic_ns"]
            - previous["host_receive_monotonic_ns"]
        ) / 1e6)
    return {
        "schema": "arduino_receive_only_diagnostic_v1",
        "port": port,
        "baud": baud,
        "sample_count": samples,
        "ignored_lines": ignored_lines,
        "first_arduino_sample_ms_mod32": rows[0]["arduino_sample_ms_mod32"],
        "last_arduino_sample_ms_mod32": rows[-1]["arduino_sample_ms_mod32"],
        "arduino_step_ms_min": min(arduino_steps),
        "arduino_step_ms_median": statistics.median(arduino_steps),
        "arduino_step_ms_max": max(arduino_steps),
        "host_receive_step_ms_min": min(host_steps),
        "host_receive_step_ms_median": statistics.median(host_steps),
        "host_receive_step_ms_max": max(host_steps),
        "clock_offset_calibrated": False,
        "one_way_delay_bounded": False,
        "force_sample_capture_time_calibrated": False,
        "command_authority": False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--receive-only", action="store_true",
                        help="diagnose one-way CSV without sending T probes")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--probes", type=int, default=32)
    parser.add_argument("--samples", type=int, default=100)
    parser.add_argument("--timeout-s", type=float, default=0.5)
    parser.add_argument("--interval-s", type=float, default=0.05)
    parser.add_argument("--warmup-s", type=float, default=2.0)
    args = parser.parse_args()
    if args.receive_only:
        result = collect_receive_only(
            args.port, baud=args.baud, samples=args.samples,
            timeout_s=args.timeout_s, warmup_s=args.warmup_s,
        )
    else:
        result = collect_probes(
            args.port, baud=args.baud, probes=args.probes,
            timeout_s=args.timeout_s, interval_s=args.interval_s,
            warmup_s=args.warmup_s,
        )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
