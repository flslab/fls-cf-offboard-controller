"""Show compression computed by the same calibration as the flight reader.

Run from the offboard repository: python -m Interaction.monitor_potentiometer
Only reads Arduino serial data; does not connect to or command the aircraft.
"""

import argparse
import math
import sys
import time

import serial

from Interaction.potentiometer_force_sensor import (
    COMPRESSION_CALIBRATION_METHOD,
    RAW_COMPRESSION_CALIBRATION,
    compression_mm_from_raw,
    parse_potentiometer_line,
)


def format_reading(line):
    text = line.decode(errors='replace').strip() if isinstance(line, bytes) else line.strip()
    if not text or text.startswith(('#', 'time_ms,')):
        return None
    sample = parse_potentiometer_line(text)
    if sample is not None:
        return f'raw={sample.raw:4d}  compression={sample.compression_mm:7.3f} mm'
    parts = text.split(',')
    if len(parts) in (5, 6):
        try:
            raw = int(parts[1])
        except ValueError:
            pass
        else:
            if 0 <= raw <= 1023 and compression_mm_from_raw(raw) is None:
                return f'raw={raw:4d}  compression=N/A (outside current calibration)'
    return f'compression=N/A (invalid serial row: {text[:100]})'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/serial0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--interval', type=float, default=.2,
                        help='seconds between printed readings (default: 0.2)')
    args = parser.parse_args(argv)
    if args.baud <= 0 or not math.isfinite(args.interval) or args.interval <= 0:
        parser.error('baud and interval must be positive and finite')

    print(f'Reading {args.port} at {args.baud} baud. Ctrl+C to stop.', flush=True)
    print('Compression uses offboard RAW_COMPRESSION_CALIBRATION; Arduino mm is ignored.', flush=True)
    print(f'Calibration curve: {COMPRESSION_CALIBRATION_METHOD}', flush=True)
    print(f'Calibration endpoints: {RAW_COMPRESSION_CALIBRATION[0]} -> '
          f'{RAW_COMPRESSION_CALIBRATION[-1]} (raw, mm)', flush=True)
    try:
        with serial.Serial(args.port, args.baud, timeout=.25, exclusive=True) as stream:
            next_print = 0.
            last_row = time.monotonic()
            next_wait_notice = last_row + 2.
            while True:
                line = stream.readline()
                now = time.monotonic()
                if not line:
                    if now >= next_wait_notice:
                        print(f'No serial data for {now - last_row:.1f}s; compression=N/A', flush=True)
                        next_wait_notice = now + 2.
                    continue
                last_row = now
                next_wait_notice = now + 2.
                reading = format_reading(line)
                if reading is not None and now >= next_print:
                    print(reading, flush=True)
                    next_print = now + args.interval
    except KeyboardInterrupt:
        print('\nStopped.', flush=True)
        return 0
    except (serial.SerialException, OSError) as error:
        print(f'Serial error: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
