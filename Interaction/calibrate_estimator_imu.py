"""Static processed-IMU calibration, optionally followed by explicit flight capture.

Run `python -m Interaction.calibrate_estimator_imu --help` for collection and
offline refitting. See Interaction/ESTIMATOR_IMU_CALIBRATION.md for fixture setup.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from copy import deepcopy
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from queue import Empty, Full, Queue
import sys
import threading
import time

import numpy as np

from Interaction.estimator_imu_calibration import (
    G, SCHEMA, analyze_file, default_plan, validate_pose, write_json,
)
from Interaction.imu_logging import fls_log_config
from Interaction.estimator_gravity_calibration import gravity_plan, check_pose_diversity

IMU_FIELDS = [f'{sensor}.{axis}' for sensor in ('acc', 'gyro') for axis in 'xyz']
MOTOR_FIELDS = [f'motor.m{i}' for i in range(1, 5)]


class TickUnwrapper:
    """CRTP log timestamps are 24-bit milliseconds, not host receipt time."""
    def __init__(self):
        self.previous = None
        self.elapsed_ms = 0

    def update(self, raw):
        if type(raw) is not int or not 0 <= raw < 1 << 24:
            raise ValueError('invalid CRTP log timestamp')
        if self.previous is not None:
            delta = (raw - self.previous) % (1 << 24)
            if not 0 < delta < (1 << 23):
                raise ValueError('duplicate/reordered IMU packet or firmware clock reset')
            self.elapsed_ms += delta
        self.previous = raw
        return self.elapsed_ms / 1000.


def check_motor_sample(data):
    values = [float(data[key]) for key in MOTOR_FIELDS]
    if not all(math.isfinite(v) and v == 0 for v in values):
        raise RuntimeError('motor output is nonzero/invalid; stop other aircraft clients')


def collect_window(next_packet, definition, duration_s, settle_s, *, clock=time.monotonic, progress=None):
    """Consume bounded callback reads; motor samples must remain fresh and zero."""
    pose = {**deepcopy(definition), 'time_s': [], 'accel_m_s2': [], 'gyro_rad_s': [],
            'cf_log_tick_ms_mod24': [], 'host_receipt_monotonic_ns': [],
            'motor_checks': []}
    if hasattr(next_packet, 'reset'):
        next_packet.reset()
    start = clock()
    recording_start = start + settle_s
    end = recording_start + duration_s
    ticks = TickUnwrapper()
    motor_received = None
    motor_tick = None
    first_t = None
    while clock() < end:
        if progress is not None:
            progress(clock()-recording_start)
        name, raw_tick, data, receipt_ns = next_packet(timeout=1.)
        received = receipt_ns / 1e9
        if not math.isfinite(received) or received < start:
            continue  # queued data from the previous pose cannot count
        if clock() - received > .2 or received > clock() + .01:
            raise RuntimeError('stale/buffered log packet; retry after fixing the connection')
        if name == 'motors':
            check_motor_sample(data)
            motor_received, motor_tick = received, raw_tick
            pose['motor_checks'].append({'cf_log_tick_ms_mod24': raw_tick,
                                         'host_receipt_monotonic_ns': receipt_ns,
                                         'outputs': [data[k] for k in MOTOR_FIELDS]})
            continue
        if name != 'imu':
            raise RuntimeError('unexpected calibration log block')
        if received < recording_start:
            continue
        if motor_received is None or not 0 <= received - motor_received <= .2:
            raise RuntimeError('fresh zero-motor telemetry unavailable')
        motor_age_ms = ((raw_tick - motor_tick + (1 << 23)) % (1 << 24)) - (1 << 23)
        if not -50 <= motor_age_ms <= 200:
            raise RuntimeError('motor telemetry is stale in the firmware clock')
        t = ticks.update(raw_tick)
        first_t = t if first_t is None else first_t
        pose['time_s'].append(t - first_t)
        pose['cf_log_tick_ms_mod24'].append(raw_tick)
        pose['host_receipt_monotonic_ns'].append(receipt_ns)
        pose['accel_m_s2'].append([float(data[f'acc.{a}']) * G for a in 'xyz'])
        pose['gyro_rad_s'].append([math.radians(float(data[f'gyro.{a}'])) for a in 'xyz'])
    return pose


@contextmanager
def radio_packets(uri):
    # Lazy imports keep offline analysis independent of radio/USB drivers.
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.log import LogConfig
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

    cflib.crtp.init_drivers()
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache=None)) as scf:
        cf = scf.cf
        deadline = time.monotonic() + 15
        while not cf.param.is_updated:
            if time.monotonic() >= deadline:
                raise TimeoutError('firmware parameter snapshot unavailable after 15s')
            time.sleep(.05)
        missing = [field for field in IMU_FIELDS + MOTOR_FIELDS
                   if field.split('.')[1] not in cf.log.toc.toc.get(field.split('.')[0], {})]
        if missing:
            raise RuntimeError('missing firmware telemetry: ' + ', '.join(missing))
        metadata = {
            'uri': uri, 'firmware_parameters_before': deepcopy(cf.param.values),
            'log_toc_fields': sorted(f'{group}.{name}' for group, fields in cf.log.toc.toc.items()
                                     for name in fields),
            'imu_fields': IMU_FIELDS, 'requested_imu_period_ms': 10,
            'requested_motor_period_ms': 50,
            'crtp_log_period_unit_ms': 1, 'cflib_period_scale': 10,
            'timestamp_basis': 'CRTP log tick (24-bit ms), not individual IMU capture time',
            'imu_atomic_sensor_snapshot': False,
            'input_units': {'acc': 'g', 'gyro': 'deg/s'},
        }
        queue = Queue(maxsize=4000)
        overflow = threading.Event()

        def push(value):
            try:
                queue.put_nowait(value)
            except Full:
                overflow.set()

        def on_data(timestamp, data, block):
            push((block.name, timestamp, data, time.monotonic_ns()))

        def on_error(block, message):
            push(RuntimeError(f'firmware log error: {message}'))

        def disconnected(uri):
            push(RuntimeError('aircraft disconnected during calibration'))

        def next_packet(timeout):
            if overflow.is_set():
                raise RuntimeError('calibration telemetry queue overflow')
            try:
                value = queue.get(timeout=timeout)
            except Empty as error:
                raise TimeoutError('no calibration telemetry received within 1s') from error
            if isinstance(value, Exception):
                raise value
            return value

        def reset():
            # Human fixture placement may take minutes. Discard that backlog;
            # within each recorded window, any new overflow remains fatal.
            while True:
                try:
                    queue.get_nowait()
                except Empty:
                    break
            overflow.clear()

        next_packet.reset = reset

        blocks = []
        cf.disconnected.add_callback(disconnected)
        try:
            for name, period, fields, kind in (
                    ('imu', 10, IMU_FIELDS, 'float'),
                    ('motors', 50, MOTOR_FIELDS, 'uint16_t')):
                block = fls_log_config(name, period)
                for field in fields:
                    block.add_variable(field, kind)
                cf.log.add_config(block)
                blocks.append(block)
                block.data_received_cb.add_callback(on_data)
                block.error_cb.add_callback(on_error)
                block.start()
            yield next_packet, metadata
        finally:
            metadata['firmware_parameters_after'] = deepcopy(cf.param.values)
            for block in blocks:
                try:
                    block.stop()
                    block.delete()
                except Exception:
                    pass  # preserve the primary link/collection error
            cf.disconnected.remove_callback(disconnected)


def collect(args, *, prompt=input, source=radio_packets, flight_runner=None):
    method = getattr(args, 'calibration_method', 'gravity_norm')
    if method not in ('gravity_norm', 'six_face'):
        raise ValueError('unknown IMU calibration method')
    gravity_norm = method == 'gravity_norm'
    six_faces = not gravity_norm and (getattr(args, 'auto_flight', False) or getattr(args, 'fit_only', False))
    plan = gravity_plan() if gravity_norm else (default_plan()[:6] if six_faces else default_plan())
    output = args.output
    output.mkdir(parents=True, exist_ok=False)
    document = {
        'schema': SCHEMA, 'status': 'collecting', 'drone_id': args.drone_id,
        'firmware_id': args.firmware_id, 'fixture_id': args.fixture_id,
        'reference_note': args.reference_note,
        'reference_source': 'gravity_magnitude' if gravity_norm else 'independent_fixture',
        'calibration_method': method,
        'sensor_frame': 'driver_processed_body', 'created_utc': datetime.now(timezone.utc).isoformat(),
        'motors_off_confirmed': False, 'poses': [], 'rejected_attempts': [],
        'command_authority': False,
    }
    path = output / 'dataset.json'
    write_json(path, document)
    try:
        print('Remove props. Close the controller/orchestrator and other aircraft clients.')
        if gravity_norm:
            print('Cage support is allowed. Collect 18 different static poses + 6 NEW validation poses.')
            print('Exact angles are not required. Support the cage without wobble; vary the tilt each time.')
            print('Fits accelerometer bias and axis scales only. Sensor/body alignment is NOT calibrated.')
        else:
            print('Use a rigid fixture aligned to BODY axes; a rounded cage is not an orientation reference.')
        # Fixture collection assumes props are removed; no typed confirmation.
        document['motors_off_confirmed'] = True
        document['motors_off_confirmation_source'] = 'fixture_collection_assumption'
        with (output / 'packets.jsonl').open('w') as journal, source(args.uri) as (read_packet, metadata):
            document['capture'] = metadata
            write_json(path, document)
            active_pose = 'warmup'

            def next_packet(timeout):
                packet = read_packet(timeout)
                journal.write(json.dumps({'pose_id': active_pose, 'block': packet[0],
                                          'cf_log_tick_ms_mod24': packet[1], 'data': packet[2],
                                          'host_receipt_monotonic_ns': packet[3]}, allow_nan=False) + '\n')
                journal.flush()
                return packet

            if hasattr(read_packet, 'reset'):
                next_packet.reset = read_packet.reset
                next_packet.reset()
            print('Keep the IMU still during startup; warming up for 10 seconds.', flush=True)
            # Drain continuously during startup to avoid a backlog and check motor output.
            warmup_end = time.monotonic() + 10
            while time.monotonic() < warmup_end:
                name, _, data, _ = next_packet(timeout=1.)
                if name == 'motors':
                    check_motor_sample(data)
            for index, definition in enumerate(plan):
                if definition['role'] == 'validation' and plan[index-1]['role'] == 'train':
                    print('Training complete. Reposition the cage/fixture for NEW VALIDATION poses.')
                while True:
                    prompt(f"[{index+1}/{len(plan)}] {definition['role']}: {definition['instruction']}. "
                           'Secure it, then press ENTER: ')
                    active_pose = definition['id']
                    print(f'Wait {args.settle_s:g}s, then collect {args.duration_s:g}s. Keep still.', flush=True)
                    pose = collect_window(next_packet, definition, args.duration_s, args.settle_s)
                    try:
                        validate_pose(pose, known_reference=not gravity_norm)
                        if gravity_norm:
                            check_pose_diversity(pose, document['poses'])
                    except ValueError as error:
                        document['rejected_attempts'].append({'error': str(error), 'pose': pose})
                        write_json(path, document)
                        print(f'Pose rejected: {error}', flush=True)
                        if prompt('Retry this pose? [Y/n]: ').strip().lower() == 'n':
                            raise ValueError('operator stopped after rejected pose')
                        continue
                    document['poses'].append(pose)
                    write_json(path, document)
                    acc = np.mean(pose['accel_m_s2'], axis=0)
                    print(f"Saved {len(pose['time_s'])} samples; mean acceleration {np.round(acc, 4)} m/s².")
                    break
            if getattr(args, 'gyro_calibration', False):
                from Interaction.estimator_gyro_calibration import rotation_plan, rotation_integral
                from Interaction.estimator_imu_calibration import fit_dataset
                provisional = {**document, 'status': 'complete'}
                static_fit = fit_dataset(provisional, require_validation=not six_faces)
                if not static_fit.get('fit_passed'):
                    raise ValueError('static fit failed; gyro calibration cannot proceed')
                document['rotations'] = []
                print('GYRO: props stay OFF. Use a fixed-axis jig with independently checked ±90° stops.')
                prompt('Confirm known axes and ±90° angle stops are ready; press ENTER to continue: ')
                for index, definition in enumerate(rotation_plan()):
                    while True:
                        prompt(f"[{index+1}/12] {definition['role']}: {definition['instruction']}. "
                               'Start at the first stop, stationary; press ENTER: ')
                        active_pose = definition['id']
                        announced = set()
                        def progress(elapsed):
                            if elapsed >= 0 and 'still' not in announced:
                                print('Keep still for 2 seconds.', flush=True)
                                announced.add('still')
                            if elapsed >= 2 and 'turn' not in announced:
                                print('ROTATE to the specified stop now; finish within 6 seconds.', flush=True)
                                announced.add('turn')
                            if elapsed >= 8 and 'end' not in announced:
                                print('HOLD STILL at the stop.', flush=True)
                                announced.add('end')
                        window = collect_window(next_packet, definition, 10., args.settle_s, progress=progress)
                        try:
                            rotation_integral(window, static_fit['gyro_residual_bias_rad_s'])
                        except ValueError as error:
                            document['rejected_attempts'].append({'error': str(error), 'rotation': window})
                            write_json(path, document)
                            print(f'Rotation rejected: {error}', flush=True)
                            if prompt('Retry this turn? [Y/n]: ').strip().lower() == 'n':
                                raise ValueError('operator stopped gyro collection')
                            continue
                        document['rotations'].append(window)
                        write_json(path, document)
                        break
        document['status'] = 'complete'
    except (Exception, KeyboardInterrupt) as error:
        document['status'] = 'incomplete'
        document['error'] = str(error) or type(error).__name__
        write_json(path, document)
        raise
    write_json(path, document)
    auto_flight = getattr(args, 'auto_flight', False)
    result = analyze_file(path, output / 'fit', require_validation=not six_faces)
    if getattr(args, 'fit_only', False):
        label = 'Gravity-magnitude fit' if gravity_norm else 'Six-face fit'
        print(f"{label} {'COMPLETE' if result.get('fit_passed') else 'REJECTED'}: {output / 'fit/report.md'}")
        if result.get('fit_passed'):
            print('Saved only. Continue with orchestrator --validate-estimator-imu; firmware unchanged.')
        else:
            for error in result['failures']:
                print(f'  {error}')
            print('Saved for diagnosis. Fit rejected; validation flight will NOT start.')
        return 0 if result.get('fit_passed') else 1
    if auto_flight:
        if not result.get('fit_passed'):
            print_result(result, output / 'fit')
            return 1
        print(f'Static fit complete: {output / "fit/candidate.json"}. Not activated or flight validated.')
        if flight_runner is None:
            from Interaction.estimator_validation_flight import run_standard_validation
            flight_runner = run_standard_validation
        flight_options={'prompt':prompt}
        if getattr(args,'flight_config',None) is None:
            flight_options['initial_position']=getattr(args,'flight_initial_position',None)
        flight = flight_runner(args.uri, output / 'fit/candidate.json', path,
                               getattr(args,'flight_config',None), output / 'flight', **flight_options)
        return 0 if flight.get('capture_completed') else 1
    print_result(result, output / 'fit')
    return 0 if result['accepted'] else 1


def print_result(result, output):
    print(f"Calibration {'PASSED' if result['accepted'] else 'REJECTED'}: {output / 'report.md'}")
    if result['accepted']:
        if result.get('calibration_method') == 'gravity_norm':
            print('Gravity magnitude validated. Sensor/body alignment rotation is NOT calibrated.')
        else:
            print(f"Alignment rotation: {result['alignment_rotation_deg']:.3f} deg")
        print(f"Calibration file: {output / 'calibration.json'}")
    else:
        for error in result['failures']:
            print(f'  {error}')
    print('Saved only. Firmware unchanged; Vicon delay and real-flight validation remain separate.')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    capture = commands.add_parser('collect', help='interactive motors-off fixture collection and validation')
    capture.add_argument('--uri', default='usb://0')
    capture.add_argument('--drone-id',required=True)
    for field in ('firmware-id', 'fixture-id', 'reference-note'):
        capture.add_argument('--' + field, default='unknown',help='optional metadata; default unknown')
    capture.add_argument('--output', required=True, type=Path, help='new session directory; never overwritten')
    capture.add_argument('--duration-s', type=float, default=4.)
    capture.add_argument('--settle-s', type=float, default=2.)
    capture.add_argument('--calibration-method', choices=('gravity_norm', 'six_face'), default='gravity_norm',
                         help='gravity_norm: cage-supported arbitrary static poses (default); six_face: exact body-aligned fixture')
    capture.add_argument('--auto-flight', action='store_true',
                         help='fit static poses, then capture a default-estimator flight for offline estimator-3 replay')
    capture.add_argument('--fit-only', action='store_true',
                         help='fit and save static calibration without starting flight; validate later through the normal orchestrator UI')
    capture.add_argument('--flight-config', type=Path,
                         help='optional advanced override of the built-in validation flight')
    capture.add_argument('--flight-initial-position',nargs=3,type=float,metavar=('X','Y','Z'),
                         help='initial marker selection seed from the existing orchestrator manifest; no YAML required')
    capture.add_argument('--gyro-calibration', action='store_true',
                         help='after static faces collect 12 known fixed-axis turns to fit and independently validate the gyro matrix')
    analyze = commands.add_parser('fit', help='offline refit of a saved fixture dataset')
    analyze.add_argument('dataset', type=Path)
    analyze.add_argument('--output', required=True, type=Path, help='new report directory')
    args = parser.parse_args(argv)
    if args.command == 'collect':
        if args.fit_only and args.auto_flight:
            parser.error('--fit-only and --auto-flight are mutually exclusive')
        if not args.auto_flight and (args.flight_config is not None or args.flight_initial_position is not None):
            parser.error('--flight-config/--flight-initial-position require --auto-flight')
        if args.flight_config is not None and args.flight_initial_position is not None:
            parser.error('choose built-in flight with --flight-initial-position or an advanced --flight-config override')
        if args.auto_flight:
            from Interaction.estimator_validation_flight import load_flight_config, builtin_flight_config
            try:
                if args.flight_config is not None:
                    load_flight_config(args.flight_config)
                else:
                    builtin_flight_config(args.flight_initial_position)
            except (ValueError, OSError) as error:
                parser.error(str(error))
        for field in ('drone_id', 'firmware_id', 'fixture_id', 'reference_note'):
            if not getattr(args, field).strip():
                parser.error(f'{field} cannot be empty')
        if not math.isfinite(args.duration_s) or not 3 <= args.duration_s <= 30:
            parser.error('--duration-s must be finite and between 3 and 30')
        if not math.isfinite(args.settle_s) or not 1 <= args.settle_s <= 30:
            parser.error('--settle-s must be finite and between 1 and 30')
    try:
        if args.command == 'collect':
            return collect(args)
        result = analyze_file(args.dataset, args.output)
        print_result(result, args.output)
        return 0 if result['accepted'] else 1
    except (ValueError, RuntimeError, TimeoutError, OSError, KeyboardInterrupt) as error:
        print(f'Calibration stopped: {str(error) or type(error).__name__}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
