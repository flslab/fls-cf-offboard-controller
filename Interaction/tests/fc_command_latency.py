"""
fc_command_latency.py
Measures the round-trip time to issue one cf.commander.send_position_setpoint
call to the flight controller.

Propellers must be OFF — this is a safe bench test.

Usage:
    python3 Interaction/tests/fc_command_latency.py
    python3 Interaction/tests/fc_command_latency.py --radio radio://0/80/2M/E7E7E7E701
    python3 Interaction/tests/fc_command_latency.py --n 500
"""

import argparse
import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
logger = logging.getLogger(__name__)

DEFAULT_URI = "usb://0"
DEFAULT_N   = 100


def run_benchmark(uri: str, n: int) -> None:
    logger.info(f"Connecting to {uri} ...")
    cflib.crtp.init_drivers(enable_serial_driver=True)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        logger.info("Connected — propellers OFF, starting benchmark")
        logger.info(f"Issuing send_position_setpoint x{n} ...")

        # Warm-up: one call before timing to avoid first-call overhead
        cf.commander.send_position_setpoint(0.0, 0.0, 0.0, 0.0)
        time.sleep(0.01)

        durations = []
        for i in range(n):
            t0 = time.perf_counter()
            cf.commander.send_position_setpoint(0.0, 0.0, 0.0, 0.0)
            t1 = time.perf_counter()
            durations.append((t1 - t0) * 1e6)   # µs

        # Stop sending setpoints
        cf.commander.send_notify_setpoint_stop()

    # ── Results ──────────────────────────────────────────────────────────────
    total_us  = sum(durations)
    avg_us    = total_us / n
    min_us    = min(durations)
    max_us    = max(durations)

    sorted_d  = sorted(durations)
    p50_us    = sorted_d[int(0.50 * n)]
    p95_us    = sorted_d[int(0.95 * n)]
    p99_us    = sorted_d[int(0.99 * n)]

    print(f"\n[fc_command_latency] Results ({n} calls)")
    print(f"  Avg   : {avg_us:>10.2f} µs")
    print(f"  Min   : {min_us:>10.2f} µs")
    print(f"  Max   : {max_us:>10.2f} µs")
    print(f"  p50   : {p50_us:>10.2f} µs")
    print(f"  p95   : {p95_us:>10.2f} µs")
    print(f"  p99   : {p99_us:>10.2f} µs")
    print(f"  Total : {total_us/1e6:>10.4f} s")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Measure send_position_setpoint latency (propellers OFF)")
    parser.add_argument("--radio", type=str, default=None,
                        help="CrazyRadio URI (default: usb://0)")
    parser.add_argument("--n", type=int, default=DEFAULT_N,
                        help=f"Number of commands to issue (default: {DEFAULT_N})")
    args = parser.parse_args()

    uri = uri_helper.uri_from_env(default=args.radio or DEFAULT_URI)
    run_benchmark(uri, args.n)