#!/usr/bin/env python3
"""Props-off delivery-rate check for the packed 1 kHz contact IMU log.

This utility only opens the radio link and subscribes to one log block.  It
does not arm the Crazyflie, send setpoints, or change a controller rate.
"""

import argparse
import math
import statistics
import threading
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from Interaction.config import CONTACT_IMU_1KHZ


DEFAULT_URI = 'radio://0/100/2M/E7E7E7E706'
DELIVERY_GATE_HZ = 950.0
MINIMUM_ACCEL_NORM_G = 0.25
MAXIMUM_SOURCE_GAP_MS = 5
MINIMUM_EPOCH_PROGRESS_PER_PACKET = 0.9
MAXIMUM_EPOCH_PROGRESS_PER_PACKET = 1.1


def _parser():
    parser = argparse.ArgumentParser(
        description='Measure the packed contactImu 1 ms log delivery rate.'
    )
    parser.add_argument('--uri', default=DEFAULT_URI)
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument(
        '--delivery-gate-hz', type=float, default=DELIVERY_GATE_HZ,
        help='Minimum received packet rate for a passing bench check.',
    )
    return parser


def _epoch_gap(previous, current):
    return (int(current) - int(previous)) & 0xFFFF


def run(uri, duration_s, delivery_gate_hz):
    if duration_s <= 0.0:
        raise ValueError('--duration must be positive')

    received = []
    errors = []
    lock = threading.Lock()
    first_packet = threading.Event()

    def on_data(timestamp, data, _logconf):
        host_time = time.monotonic()
        values = tuple(float(data[name]) for name in CONTACT_IMU_1KHZ
                       if name != 'log_period_ms' and
                       name != 'contactImu.epoch')
        if not all(math.isfinite(value) for value in values):
            errors.append('non-finite contactImu value')
        with lock:
            received.append((
                host_time,
                int(data['contactImu.epoch']),
                values,
            ))
        first_packet.set()

    def on_error(_logconf, message):
        errors.append(str(message))
        first_packet.set()

    cflib.crtp.init_drivers()
    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(uri, cf=cf) as scf:
        toc_storage_type = scf.cf.log.toc.toc['contactImu']['gx'].ctype
        print(f'contactImu.gx TOC storage type = {toc_storage_type}')
        # The deployed firmware interprets the encoded legacy period byte as
        # milliseconds. cflib divides this argument by ten, so 10 encodes 1.
        logconf = LogConfig(name='CONTACT_IMU_RATE', period_in_ms=10)
        for name, metadata in CONTACT_IMU_1KHZ.items():
            if name != 'log_period_ms':
                logconf.add_variable(name, metadata['type'])
        if logconf.period != 1:
            raise RuntimeError(
                f'expected encoded period 1, got {logconf.period}'
            )
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(on_data)
        logconf.error_cb.add_callback(on_error)
        start = time.monotonic()
        logconf.start()
        first_packet.wait(timeout=min(2.0, duration_s))
        while time.monotonic() - start < duration_s and not errors:
            time.sleep(0.01)
        logconf.stop()

    if errors:
        raise RuntimeError('; '.join(errors))
    if not received:
        raise RuntimeError('no contactImu packets received')

    host_times = [sample[0] for sample in received]
    epochs = [sample[1] for sample in received]
    payloads = [sample[2] for sample in received]
    accel_norms = [
        math.sqrt(sum(value * value for value in payload[3:6]))
        for payload in payloads
    ]
    plausible_accel_samples = sum(
        norm >= MINIMUM_ACCEL_NORM_G for norm in accel_norms
    )
    all_zero_samples = sum(
        all(value == 0.0 for value in payload) for payload in payloads
    )
    elapsed = host_times[-1] - host_times[0] if len(host_times) > 1 else 0.0
    rate_hz = (len(received) - 1) / elapsed if elapsed > 0.0 else 0.0
    gaps = [_epoch_gap(left, right) for left, right in zip(epochs, epochs[1:])]
    missed_events = [gap for gap in gaps if gap > 1]
    stagnant_events = [gap for gap in gaps if gap == 0]
    epoch_advanced = len(set(epochs)) > 1
    epoch_progress_ms = _epoch_gap(epochs[0], epochs[-1])
    epoch_progress_per_packet = (
        epoch_progress_ms / (len(epochs) - 1) if len(epochs) > 1 else 0.0
    )
    source_timing_valid = (
        epoch_advanced and
        max(gaps, default=0) <= MAXIMUM_SOURCE_GAP_MS and
        MINIMUM_EPOCH_PROGRESS_PER_PACKET <= epoch_progress_per_packet <=
        MAXIMUM_EPOCH_PROGRESS_PER_PACKET
    )
    intervals_ms = [
        (right - left) * 1000.0
        for left, right in zip(host_times, host_times[1:])
    ]
    max_gap_ms = max(intervals_ms, default=0.0)
    median_gap_ms = statistics.median(intervals_ms) if intervals_ms else 0.0
    payload_valid = (
        plausible_accel_samples > 0 and all_zero_samples < len(received)
    )
    passed = (
        rate_hz >= delivery_gate_hz and source_timing_valid and payload_valid
    )

    print('CONTACT_IMU_1KHZ cflib encoded period = 1')
    print(f'packets={len(received)} rate={rate_hz:.1f} Hz')
    print(
        f'epoch_gaps_gt_1={len(missed_events)} '
        f'max_epoch_gap={max(gaps, default=0)} ms '
        f'epoch_stagnant={len(stagnant_events)} '
        f'epoch_first={epochs[0]} epoch_last={epochs[-1]} '
        f'epoch_progress_per_packet={epoch_progress_per_packet:.3f}'
    )
    print(
        f'host_interval_median={median_gap_ms:.3f} ms '
        f'host_interval_max={max_gap_ms:.3f} ms'
    )
    print(
        f'accel_norm_median={statistics.median(accel_norms):.3f} g '
        f'plausible_accel_samples={plausible_accel_samples} '
        f'all_zero_payloads={all_zero_samples}'
    )
    print('RESULT=' + ('PASS' if passed else 'FAIL'))
    return 0 if passed else 2


def main():
    args = _parser().parse_args()
    try:
        return run(args.uri, args.duration, args.delivery_gate_hz)
    except Exception as exc:  # Make radio/TOC failures concise at the bench.
        print(f'RESULT=ERROR: {exc}')
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
