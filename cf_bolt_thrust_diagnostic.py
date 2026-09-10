#!/usr/bin/env python3
"""Bench-only Crazyflie Bolt USB, motor, and battery diagnostic.

The default mode is read-only. Motor output requires both ``--run-motors``
and ``--bench-secured``. Motor commands use the normal low-level commander at
50 Hz so the firmware setpoint watchdog remains active if USB disappears.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import math
from pathlib import Path
import statistics
import subprocess
import sys
import threading
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


USB_SYSFS = Path("/sys/bus/usb/devices/2-1")
MAX_THRUST = 40000


def parse_levels(value: str) -> list[int]:
    try:
        levels = [int(item.strip()) for item in value.split(",") if item.strip()]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("levels must be comma-separated integers") from exc
    if not levels:
        raise argparse.ArgumentTypeError("at least one thrust level is required")
    if any(level < 0 or level > MAX_THRUST for level in levels):
        raise argparse.ArgumentTypeError(
            f"every thrust level must be between 0 and {MAX_THRUST}"
        )
    return levels


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(
        description=(
            "Diagnose Bolt USB resets and battery sag. Read-only unless both "
            "--run-motors and --bench-secured are supplied."
        )
    )
    result.add_argument("--uri", default="usb://0")
    result.add_argument("--duration-s", type=float, default=30.0,
                        help="read-only monitoring duration")
    result.add_argument("--run-motors", action="store_true",
                        help="enable the bounded thrust sequence")
    result.add_argument("--bench-secured", action="store_true",
                        help="confirm the vehicle cannot lift, translate, or rotate")
    result.add_argument("--levels", type=parse_levels,
                        default=parse_levels("0,20000,30000,40000"))
    result.add_argument("--dwell-s", type=float, default=2.0)
    result.add_argument("--rest-s", type=float, default=1.0)
    result.add_argument("--countdown-s", type=int, default=5)
    result.add_argument("--output-dir", type=Path, default=Path("/tmp"))
    return result


def finite_positive(name: str, value: float) -> None:
    if not math.isfinite(value) or value <= 0:
        raise SystemExit(f"{name} must be finite and positive")


def ensure_no_controller() -> None:
    check = subprocess.run(
        ["pgrep", "-af", "[p]ython.*controller.py"],
        text=True,
        capture_output=True,
        check=False,
    )
    if check.returncode == 0 and check.stdout.strip():
        raise SystemExit(
            "Refusing to run while controller.py is active:\n" + check.stdout.strip()
        )


def pi_throttle_state() -> str:
    try:
        return subprocess.check_output(
            ["vcgencmd", "get_throttled"], text=True, timeout=1.0
        ).strip()
    except (OSError, subprocess.SubprocessError):
        return "unavailable"


def main() -> int:
    args = parser().parse_args()
    finite_positive("duration-s", args.duration_s)
    finite_positive("dwell-s", args.dwell_s)
    if not math.isfinite(args.rest_s) or args.rest_s < 0:
        raise SystemExit("rest-s must be finite and nonnegative")
    if args.countdown_s < 0 or args.countdown_s > 30:
        raise SystemExit("countdown-s must be between 0 and 30")
    if args.run_motors and not args.bench_secured:
        raise SystemExit(
            "Motor test refused: physically restrain the vehicle, clear the "
            "area, then add --bench-secured"
        )
    ensure_no_controller()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    tag = datetime.now().strftime("%Y%m%dT%H%M%S")
    csv_path = args.output_dir / f"cf_bolt_thrust_diag_{tag}.csv"
    event_path = args.output_dir / f"cf_bolt_thrust_diag_{tag}.events.log"

    link_lost = threading.Event()
    finished = threading.Event()
    rows_lock = threading.Lock()
    step_voltages: list[float] = []
    current = {"phase": "connecting", "thrust": 0, "supervisor_info": None}
    events = event_path.open("w", encoding="utf-8", buffering=1)
    csv_file = csv_path.open("w", encoding="utf-8", newline="", buffering=1)
    writer = csv.writer(csv_file)
    writer.writerow([
        "host_time_s", "cf_timestamp_ms", "phase", "commanded_thrust",
        "pm.vbat", "motor.m1", "motor.m2", "motor.m3", "motor.m4",
        "supervisor.info", "usb_present", "usb_device_number",
    ])

    def record_event(message: str) -> None:
        line = f"{time.time():.6f} {message}"
        print(line, flush=True)
        events.write(line + "\n")

    def device_number() -> str:
        try:
            return (USB_SYSFS / "devnum").read_text(encoding="utf-8").strip()
        except OSError:
            return ""

    def watch_usb() -> None:
        previous = None
        while not finished.wait(0.02):
            present = USB_SYSFS.exists()
            state = (present, device_number() if present else "")
            if state != previous:
                record_event(f"USB_STATE present={present} device={state[1] or 'none'}")
                previous = state

    def on_connection_lost(*callback_args) -> None:
        record_event("CONNECTION_LOST " + " | ".join(map(str, callback_args)))
        link_lost.set()

    def on_log_error(log_conf, message) -> None:
        record_event(f"LOG_ERROR group={log_conf.name} message={message}")
        link_lost.set()

    def on_data(timestamp, data, _log_conf) -> None:
        voltage = data.get("pm.vbat")
        with rows_lock:
            info = data.get("supervisor.info")
            if isinstance(info, int):
                current["supervisor_info"] = info
            if isinstance(voltage, (int, float)) and math.isfinite(voltage):
                step_voltages.append(float(voltage))
            present = USB_SYSFS.exists()
            writer.writerow([
                f"{time.time():.6f}", timestamp, current["phase"],
                current["thrust"], voltage,
                data.get("motor.m1"), data.get("motor.m2"),
                data.get("motor.m3"), data.get("motor.m4"),
                info,
                int(present), device_number() if present else "",
            ])

    def wait_for_supervisor_bit(mask: int, expected: bool, timeout_s: float) -> int:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if link_lost.is_set():
                raise RuntimeError("connection lost while waiting for supervisor")
            with rows_lock:
                info = current["supervisor_info"]
            if isinstance(info, int) and bool(info & mask) is expected:
                return info
            time.sleep(0.02)
        raise RuntimeError(
            f"supervisor bit 0x{mask:x} did not become {expected}; "
            f"last info={current['supervisor_info']!r}"
        )

    def send_for(commander, thrust: int, duration_s: float, phase: str) -> None:
        current.update(phase=phase, thrust=thrust)
        with rows_lock:
            step_voltages.clear()
        started = time.monotonic()
        next_send = started
        while time.monotonic() - started < duration_s:
            if link_lost.is_set():
                raise RuntimeError("USB/CRTP connection was lost")
            commander.send_setpoint(0.0, 0.0, 0.0, thrust)
            next_send += 0.02
            time.sleep(max(0.0, next_send - time.monotonic()))
        with rows_lock:
            voltages = list(step_voltages)
        summary = "no voltage samples"
        if voltages:
            summary = (
                f"vbat_min={min(voltages):.3f} "
                f"vbat_median={statistics.median(voltages):.3f} "
                f"vbat_max={max(voltages):.3f}"
            )
        record_event(
            f"STEP_COMPLETE phase={phase} thrust={thrust} {summary} "
            f"pi_power={pi_throttle_state()}"
        )

    usb_thread = threading.Thread(target=watch_usb, daemon=True)
    usb_thread.start()
    cf = Crazyflie(rw_cache="/tmp/cf_bolt_thrust_diag_cache")
    cf.connection_lost.add_callback(on_connection_lost)
    # This firmware currently delivers a 100 ms request at about 100 Hz over
    # direct USB. That is fast enough to see battery sag without reproducing
    # the much heavier flight-log load inside the diagnostic itself.
    log_config = LogConfig(name="bolt_diag", period_in_ms=100)
    for name, kind in (
        ("pm.vbat", "float"),
        ("motor.m1", "uint16_t"), ("motor.m2", "uint16_t"),
        ("motor.m3", "uint16_t"), ("motor.m4", "uint16_t"),
        ("supervisor.info", "uint16_t"),
    ):
        log_config.add_variable(name, kind)
    log_config.data_received_cb.add_callback(on_data)
    log_config.error_cb.add_callback(on_log_error)

    record_event(
        f"START uri={args.uri} motor_test={args.run_motors} "
        f"levels={args.levels if args.run_motors else []} "
        f"pi_power={pi_throttle_state()}"
    )
    exit_code = 0
    cflib.crtp.init_drivers()
    try:
        with SyncCrazyflie(args.uri, cf=cf) as scf:
            scf.cf.log.add_config(log_config)
            log_config.start()
            current["phase"] = "monitor"
            record_event("CONNECTED")
            if args.run_motors:
                record_event(
                    "MOTOR_TEST_PENDING vehicle_must_be_physically_secured; "
                    f"countdown={args.countdown_s}s"
                )
                for remaining in range(args.countdown_s, 0, -1):
                    record_event(f"COUNTDOWN {remaining}")
                    if link_lost.wait(1.0):
                        raise RuntimeError("connection lost during countdown")
                info = wait_for_supervisor_bit(0x0001, True, 2.0)
                record_event(f"CAN_BE_ARMED supervisor_info=0x{info:04x}")
                scf.cf.platform.send_arming_request(True)
                record_event("ARM_REQUESTED")
                info = wait_for_supervisor_bit(0x0002, True, 2.0)
                record_event(f"ARMED supervisor_info=0x{info:04x}")
                for index, thrust in enumerate(args.levels):
                    send_for(
                        scf.cf.commander, thrust, args.dwell_s,
                        f"thrust_{thrust}",
                    )
                    if args.rest_s and index + 1 < len(args.levels):
                        send_for(scf.cf.commander, 0, args.rest_s, "rest")
            else:
                deadline = time.monotonic() + args.duration_s
                while time.monotonic() < deadline:
                    if link_lost.wait(min(0.1, deadline - time.monotonic())):
                        raise RuntimeError("USB/CRTP connection was lost")
            record_event("TEST_COMPLETE")
    except KeyboardInterrupt:
        record_event("INTERRUPTED")
        exit_code = 130
    except Exception as exc:
        record_event(f"TEST_FAILED type={type(exc).__name__} error={exc}")
        exit_code = 1
    finally:
        # Best-effort cleanup. If the link is already gone, the firmware's
        # normal commander watchdog remains the independent motor cutoff.
        if args.run_motors and not link_lost.is_set():
            try:
                for _ in range(10):
                    cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
                    time.sleep(0.02)
                record_event("ZERO_THRUST_SENT")
                cf.platform.send_arming_request(False)
                record_event("DISARM_REQUESTED")
            except Exception as exc:
                record_event(f"STOP_FAILED type={type(exc).__name__} error={exc}")
                exit_code = 1
        try:
            log_config.stop()
        except Exception:
            pass
        finished.set()
        usb_thread.join(timeout=1.0)
        csv_file.close()
        events.close()
        print(f"CSV={csv_path}\nEVENTS={event_path}", flush=True)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
