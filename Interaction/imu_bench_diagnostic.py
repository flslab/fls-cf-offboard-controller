"""Motors-off, read-only IMU bench capture. No flight/controller imports."""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import json
import math
import multiprocessing as mp
from pathlib import Path
from queue import Empty, Full, Queue
import threading
import time

from Interaction.imu_logging import fls_log_config

FIELDS = [f'contactImu.{prefix}{axis}' for prefix in ('g', 'a') for axis in 'xyz']
MOTORS = [f'motor.m{i}' for i in range(1, 5)]
PLAN = [
    ('static_before', 20, '放稳机身，不触碰，记录静止基线。'),
    ('slow_roll', 30, '手持机身，缓慢 roll 倾斜约 ±10°，往返几次。'),
    ('gentle_taps', 30, '稳定支撑机身，轻推接触头后松开 5–6 次，每次间隔约 3 秒。'),
    ('static_after', 20, '恢复初始姿态，放稳机身，不触碰。'),
]


def save_json(path, value):
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    temp.replace(path)


@contextmanager
def radio_packets(uri, period_ms):
    """Callbacks only queue data; the consumer persists it outside the radio thread."""
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

    packets = Queue(maxsize=12000)
    overflow = threading.Event()

    def push(row):
        try:
            packets.put_nowait(row)
        except Full:
            overflow.set()

    def on_data(tick, data, block):
        push({'kind': block.name, 'cf_log_tick_ms_mod24': int(tick),
              'host_monotonic_ns': time.monotonic_ns(), 'host_unix_s': time.time(),
              'data': dict(data)})

    def on_error(block, message):
        push(RuntimeError(f'{block.name}: {message}'))

    def disconnected(*_):
        push(RuntimeError('aircraft disconnected'))

    def read(timeout=.1):
        if overflow.is_set():
            raise RuntimeError('IMU queue overflow; capture incomplete')
        try:
            row = packets.get(timeout=timeout)
        except Empty:
            return None
        if isinstance(row, Exception):
            raise row
        return row

    cflib.crtp.init_drivers()
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache=None)) as scf:
        cf = scf.cf
        missing = [f for f in FIELDS + ['contactImu.epoch'] + MOTORS
                   if f.split('.')[1] not in cf.log.toc.toc.get(f.split('.')[0], {})]
        if missing:
            raise RuntimeError('Firmware lacks required telemetry: ' + ', '.join(missing))
        metadata = {'uri': uri, 'requested_period_ms': period_ms,
                    'transport': 'FP16 body IMU; uint16 producer time in milliseconds',
                    'sensor_input': 'firmware-filtered; not raw chip/pre-filter data',
                    'crtp_period_encoding': 'FLS firmware, 1 ms per period byte',
                    'fields': FIELDS + ['contactImu.epoch'] + MOTORS,
                    'motor_checks_in_imu_packet': True,
                    'toc_storage_types': {f: cf.log.toc.toc[f.split('.')[0]][f.split('.')[1]].ctype
                                          for f in FIELDS}}
        blocks = []
        cf.disconnected.add_callback(disconnected)
        try:
            for name, period, fields in (
                ('imu', period_ms, [(f, 'FP16') for f in FIELDS] + [('contactImu.epoch', 'uint16_t')]
                 + [(f, 'uint16_t') for f in MOTORS]),
            ):
                block = fls_log_config(name, period)
                for field, dtype in fields:
                    block.add_variable(field, dtype)
                cf.log.add_config(block)
                blocks.append(block)
                block.data_received_cb.add_callback(on_data)
                block.error_cb.add_callback(on_error)
                block.start()
            yield read, metadata
        finally:
            for block in blocks:
                try:
                    block.stop()
                    block.delete()
                except Exception:
                    pass  # Retain the original capture failure.
            cf.disconnected.remove_callback(disconnected)


class Recorder:
    """Continuously drain telemetry, including while the operator prepares a stage."""
    def __init__(self, path, read):
        self.path, self.read = path, read
        self.stage = 'preparation'
        self.stop = threading.Event()
        self.ready = threading.Event()
        self.error = None
        self.counts = {'imu': 0, 'motors': 0, 'motor_checks': 0}
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def check(self):
        if self.error:
            raise RuntimeError(self.error)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)
        if self.thread.is_alive():
            raise RuntimeError('recorder did not stop; session is incomplete')
        self.check()

    def _run(self):
        start = last_flush = time.monotonic()
        last_motor = last_imu = None
        last_epoch = last_progress = None
        motor_tick = None
        try:
            with self.path.open('x', buffering=1) as file:
                while not self.stop.is_set():
                    row = self.read(.1)
                    now = time.monotonic()
                    if row is not None:
                        # Persist before validation, including the packet that caused failure.
                        row = {**row, 'stage': self.stage}
                        file.write(json.dumps(row, allow_nan=True) + '\n')
                        received = row['host_monotonic_ns'] / 1e9
                        if not -.01 <= now - received <= .3:
                            raise RuntimeError('stale/buffered telemetry')
                        data, kind = row['data'], row['kind']
                        if kind == 'motors' or (kind == 'imu' and all(f in data for f in MOTORS)):
                            values = [float(data[f]) for f in MOTORS]
                            if not all(math.isfinite(v) and v == 0 for v in values):
                                raise RuntimeError('motor output nonzero/invalid; close other aircraft clients')
                            last_motor, motor_tick = received, row['cf_log_tick_ms_mod24']
                            self.counts['motor_checks'] += 1
                        if kind == 'imu':
                            if not all(math.isfinite(float(data[f])) for f in FIELDS):
                                raise RuntimeError('non-finite IMU payload')
                            epoch = data['contactImu.epoch']
                            if not isinstance(epoch, int) or not 0 <= epoch < 65536:
                                raise RuntimeError('invalid producer epoch')
                            progressing = False
                            if last_epoch is not None:
                                delta = (epoch-last_epoch) % 65536
                                if delta >= 32768:
                                    raise RuntimeError('producer clock reset or reordered IMU packet')
                                progressing = delta > 0
                            if last_epoch is None or progressing:
                                last_progress = received
                            last_epoch = epoch
                            last_imu = received
                            if last_motor is not None and 0 <= received-last_motor <= .2:
                                skew = ((row['cf_log_tick_ms_mod24'] - motor_tick + 2**23) % 2**24) - 2**23
                                if not -50 <= skew <= 200:
                                    raise RuntimeError('motor telemetry stale in firmware clock')
                                accel_norm = math.sqrt(sum(float(data[f'contactImu.a{a}'])**2 for a in 'xyz'))
                                if progressing and accel_norm >= .25:
                                    self.ready.set()
                        elif kind != 'motors':
                            raise RuntimeError('unexpected telemetry block')
                        self.counts[kind] += 1
                    if now - start > 1.0:
                        if last_motor is None or now - last_motor > .3:
                            raise RuntimeError('fresh zero-motor telemetry unavailable')
                        if last_imu is None or now - last_imu > .3:
                            raise RuntimeError('IMU telemetry stopped')
                        if last_progress is None or now - last_progress > .3:
                            raise RuntimeError('IMU producer time stopped advancing')
                    if now - last_flush >= .5:
                        file.flush()
                        last_flush = now
        except Exception as exc:
            self.error = str(exc)


def wait_for_enter(recorder, message, prompt=input):
    """Operator input never blocks telemetry recording or error handling."""
    answer = Queue()
    def ask():
        try:
            answer.put(prompt(message))
        except BaseException as exc:
            answer.put(exc)
    threading.Thread(target=ask, daemon=True).start()
    while True:
        recorder.check()
        try:
            value = answer.get(timeout=.1)
        except Empty:
            continue
        if isinstance(value, BaseException):
            raise value
        return value


def vicon_worker(host, path, stop):
    """Optional raw optical record; no marker identity or estimator feedback."""
    try:
        import motioncapture
        mc = motioncapture.connect('vicon', {'hostname': host})
        with Path(path).open('x', buffering=1) as file:
            while not stop.is_set():
                mc.waitForNextFrame()
                row = {'host_monotonic_ns': time.monotonic_ns(), 'host_unix_s': time.time(),
                       'timestamp_basis': 'host after frame wait; not camera exposure',
                       'point_cloud_m': [[float(v) for v in p] for p in mc.pointCloud]}
                if callable(getattr(mc, 'frameMetadata', None)):
                    try:
                        row['source_metadata'] = dict(mc.frameMetadata())
                    except Exception as exc:
                        row['metadata_error'] = str(exc)
                file.write(json.dumps(row) + '\n')
    except Exception as exc:
        save_json(Path(path).with_suffix('.error.json'), {'error': str(exc)})


def capture(args, *, prompt=input, source=radio_packets):
    output = args.output
    output.mkdir(parents=True, exist_ok=False)
    manifest = {'schema': 'imu_bench_v1', 'status': 'preparing', 'drone_id': args.drone_id,
                'created_utc': datetime.now(timezone.utc).isoformat(), 'uri': args.uri,
                'command_authority': False, 'calibration_applied': False,
                'props_removed_confirmed': False, 'stages': [],
                'vicon_requested': bool(args.vicon_host),
                'geometry_m': {'imu_z_from_com': -.01, 'marker_z_from_com': .06,
                               'basis': 'approximate user measurements; not fitted'}}
    save_json(output / 'session.json', manifest)
    recorder = process = stop_vicon = None
    error = None
    try:
        print('拆除桨叶，关闭 controller/orchestrator 和其他飞机客户端。电机必须保持不转。')
        if prompt('确认已拆桨，输入 PROPS_OFF：').strip() != 'PROPS_OFF':
            raise RuntimeError('props-off confirmation not received')
        manifest['props_removed_confirmed'] = True
        if args.vicon_host:
            ctx = mp.get_context('spawn')
            stop_vicon = ctx.Event()
            process = ctx.Process(target=vicon_worker,
                                  args=(args.vicon_host, str(output/'vicon.jsonl'), stop_vicon), daemon=True)
            process.start()
        with source(args.uri, args.period_ms) as (read, metadata):
            manifest['telemetry'] = metadata
            recorder = Recorder(output/'packets.jsonl', read)
            recorder.start()
            try:
                deadline = time.monotonic() + 5
                while not recorder.ready.wait(.1):
                    recorder.check()
                    if time.monotonic() > deadline:
                        raise TimeoutError('no fresh IMU plus zero-motor telemetry within 5 s')
                manifest['status'] = 'recording'
                for name, duration, instruction in PLAN:
                    wait_for_enter(recorder, f'\n{name}: {instruction}\n准备好后按 ENTER：', prompt)
                    recorder.check()
                    entry = {'name': name, 'duration_requested_s': duration,
                             'start_host_monotonic_ns': time.monotonic_ns(), 'completed': False}
                    manifest['stages'].append(entry)
                    recorder.stage = name
                    save_json(output/'session.json', manifest)
                    deadline = time.monotonic() + duration
                    print(f'正在记录 {duration} 秒 …', flush=True)
                    while time.monotonic() < deadline:
                        recorder.check()
                        time.sleep(.1)
                    entry['end_host_monotonic_ns'] = time.monotonic_ns()
                    entry['completed'] = True
                    recorder.stage = 'preparation'
                    save_json(output/'session.json', manifest)
                manifest['status'] = 'completed'
            finally:
                recorder.close()
    except BaseException as exc:
        error = exc
        manifest['status'] = 'interrupted' if isinstance(exc, (KeyboardInterrupt, EOFError)) else 'failed'
        manifest['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        if recorder:
            manifest['packet_counts'] = recorder.counts
        if process:
            stop_vicon.set()
            process.join(timeout=2)
            forced = process.is_alive()
            if forced:
                process.terminate()
                process.join(timeout=2)
            manifest['vicon'] = {'shutdown_wait_timed_out': forced,
                                 'file': 'vicon.jsonl', 'marker_identity': 'unassigned raw point cloud',
                                 'has_recorded_bytes': (output/'vicon.jsonl').exists() and (output/'vicon.jsonl').stat().st_size > 0,
                                 'error_file_present': (output/'vicon.error.json').exists()}
        save_json(output/'session.json', manifest)
    if (output/'packets.jsonl').exists() and not getattr(args, 'no_report', False):
        try:
            from Interaction.imu_bench_report import analyze
            report = analyze(output)
            qualified = sum(stage['high_rate_qualified'] for stage in report['stages'])
            print(f'1 kHz 数据质量合格阶段：{qualified}/4；请查看 report.html。')
        except Exception as exc:
            manifest['analysis_error'] = str(exc)
            save_json(output/'session.json', manifest)
            print(f'数据已保留；报告生成失败：{exc}')
    print(f'采集状态：{manifest["status"]}；结果：{output}')
    if error:
        print(manifest['error'])
    return 0 if manifest['status'] == 'completed' and 'analysis_error' not in manifest else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    cap = sub.add_parser('collect', help='interactive props-off capture; never arms or sends setpoints')
    cap.add_argument('--uri', required=True, help='explicit aircraft URI; FLS custom firmware required')
    cap.add_argument('--drone-id', default='lb11')
    cap.add_argument('--output', type=Path, required=True, help='new session directory, never overwritten')
    cap.add_argument('--period-ms', type=int, choices=(1, 2, 5, 10), default=1)
    cap.add_argument('--vicon-host', help='optional Vicon raw point-cloud capture on the same host clock')
    cap.add_argument('--no-report', action='store_true',
                     help='save capture only; generate report on another machine with the analyze command')
    report = sub.add_parser('analyze', help='offline report; radio drivers are not loaded')
    report.add_argument('session', type=Path)
    args = parser.parse_args()
    if args.command == 'collect':
        return capture(args)
    from Interaction.imu_bench_report import analyze
    analyze(args.session)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
