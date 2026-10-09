"""Read frozen firmware RAM trace; no motor/commander or estimator-enable calls.

`record` is a props-off bench collector. Flight capture uses the existing bounded
validation flight with an optional ram_trace block; download occurs after stop.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import threading
import time
import numpy as np
from Interaction.estimator_imu_calibration import write_json

HEADER = struct.Struct('<14I')
RECORD = struct.Struct('<III7f')
MAGIC = 0x33525445

def validate_interaction_trace(config, coast_mode):
    if config is None:
        return None
    if not isinstance(config,dict) or set(config)-{'hz','post_release_ms'}:
        raise ValueError('level_coast.ram_trace supports hz and post_release_ms')
    result={'hz':config.get('hz',1000),'post_release_ms':config.get('post_release_ms',200)}
    if (type(result['hz']) is not int or result['hz'] not in (250,500,1000)
            or type(result['post_release_ms']) is not int or not 0 <= result['post_release_ms'] <= 5000):
        raise ValueError('invalid interaction RAM recording rate/post-release interval')
    if coast_mode=='scurve':
        raise ValueError('RAM capture runs only default estimator 2; select orientation or position coast')
    return result


def prepare_interaction_recorder(controller):
    config=(controller.mission or {}).get('Interaction',{}).get('config',{})
    if config.get('behavior')!='level_coast':
        return
    options=config.get('level_coast') or {}
    settings=validate_interaction_trace(options.get('ram_trace'),options.get('coast_command_mode',options.get('command_mode','orientation')))
    if settings is None:
        return
    recorder=Recorder(controller.cf)
    recorder.prepare_interaction(**{'hz':settings['hz'],'post_release_ms':settings['post_release_ms']})
    controller.cf._interaction_ram_recorder=recorder
    controller.cf._interaction_ram_started=False
    controller.cf._interaction_ram_released=False


def interaction_step(owner, phase, released):
    cf=owner.cf
    recorder=getattr(cf,'_interaction_ram_recorder',None)
    if recorder is None:
        return
    if not cf._interaction_ram_started and phase in ('ready','contact'):
        recorder.start_async(recorder.prepared[0],0)
        cf._interaction_ram_started=True
        owner._log_event('RAM Trace Started',{'phase':phase})
    if cf._interaction_ram_started and released and not cf._interaction_ram_released:
        recorder.release_async()
        cf._interaction_ram_released=True
        owner._log_event('RAM Trace Release Trigger',{'phase':phase})


def finish_interaction_recorder(controller):
    recorder=getattr(controller.cf,'_interaction_ram_recorder',None)
    if recorder is None or not getattr(controller.cf,'_interaction_ram_started',False):
        return
    recorder.freeze_async()
    if getattr(controller,'flying',True):
        raise RuntimeError('RAM frozen but not downloaded while landing is unconfirmed; keep power for recovery')
    recorder.freeze()
    recorder.verify_stopped()
    output=Path(controller.args.log_dir)/(controller.args.tag+'_ram_trace')
    raw=recorder.download()
    report=save(raw,output,recorder.last_download_diagnostics)
    print(f'RAM trace saved: {output} ({report["retained_duration_s"]:.3f}s retained)',flush=True)
    return report

def validate_flight_trace(config):
    if not isinstance(config, dict) or set(config)-{'phase','hz','duration_ms','offset_s'}:
        raise ValueError('ram_trace requires phase, hz, duration_ms and optional offset_s')
    if not {'phase','hz','duration_ms'}.issubset(config):
        raise ValueError('ram_trace requires phase, hz and duration_ms')
    durations = {'hover_start': 5., 'hover_end': 5.}
    durations.update({axis+suffix: 3. for axis in ('+X','-X','+Y','-Y')
                      for suffix in ('_out','_hold','_return','_center')})
    offset=config.get('offset_s',0)
    if (config['phase'] not in durations or type(config['hz']) is not int
            or config['hz'] not in (250,500,1000) or type(config['duration_ms']) is not int
            or not 1 <= config['duration_ms'] <= 30000 or type(offset) not in (int,float)
            or not np.isfinite(offset) or not 0 <= offset < durations[config['phase']]
            or offset+config['duration_ms']/1000 > durations[config['phase']]):
        raise ValueError('invalid RAM trace rate/duration or capture does not fit the selected flight phase')

def decode(raw):
    if len(raw) < HEADER.size:
        raise ValueError('truncated RAM trace header')
    names = ('magic', 'version', 'record_bytes', 'capacity', 'count', 'overwritten',
             'session', 'status', 'imu_hz', 'start_us', 'stop_us', 'unmatched_imu',
             'queue_rejected', 'decimated')
    header = dict(zip(names, HEADER.unpack_from(raw)))
    if (header['magic'] != MAGIC or header['version'] != 1 or header['record_bytes'] != RECORD.size
            or header['status'] != 2 or not 0 < header['count'] <= header['capacity'] <= 65536
            or header['imu_hz'] not in (250, 500, 1000)):
        raise ValueError('unsupported/not-frozen RAM trace')
    if len(raw) != HEADER.size + header['count']*RECORD.size:
        raise ValueError('RAM trace size/count mismatch')
    rows, receipts, imu_stamps, previous = [], [], [], None
    elapsed = 0
    rejected_inputs = 0
    for offset in range(HEADER.size, len(raw), RECORD.size):
        kind_flags, receipt, sample, *data = RECORD.unpack_from(raw, offset)
        kind = kind_flags & 255
        if kind not in (1, 2, 3, 4, 5) or not np.isfinite(data).all():
            raise ValueError('invalid RAM trace record')
        if previous is not None:
            delta = (receipt-previous) % (1 << 32)
            if delta >= 1 << 31:
                raise ValueError('reordered RAM receipt clock')
            elapsed += delta
        previous = receipt
        receipts.append(elapsed)
        lag = (receipt-sample) % (1 << 32)
        if lag > 100000:
            raise ValueError('acquisition timestamp not causal or >100ms old')
        row = {'received_s': elapsed/1e6, 'device_time_us': elapsed-lag,
               'phase': 'ram_capture', 'cf_log_tick_ms_mod24': (sample//1000) % (1 << 24)}
        if kind == 1:
            if kind_flags & 0x300 != 0x300:
                rejected_inputs += 1
                continue
            row['group'] = 'imu'
            row['data'] = {**dict(zip(('gyro.x','gyro.y','gyro.z'), data[:3])),
                           **dict(zip(('acc.x','acc.y','acc.z'), data[3:6]))}
            if imu_stamps and row['device_time_us'] <= imu_stamps[-1]:
                raise ValueError('duplicate/reordered RAM acquisition clock')
            imu_stamps.append(row['device_time_us'])
        elif kind == 2:
            if not kind_flags & 0x100:
                rejected_inputs += 1
                continue
            row.update(group='vicon', data={'position_m': data[:3], 'stddev_m': data[3]})
        elif kind == 3:
            q = np.asarray(data[:4])
            if not .9 <= np.linalg.norm(q) <= 1.1:
                raise ValueError('invalid default estimator quaternion')
            row.update(group='kf', data=dict(zip(('kalman.q0','kalman.q1','kalman.q2','kalman.q3'), data)))
        elif kind == 4:
            row.update(group='state', data=dict(zip(('stateEstimate.x','stateEstimate.y','stateEstimate.z',
                       'stateEstimate.vx','stateEstimate.vy','stateEstimate.vz'), data)))
        else:
            row.update(group='event', data={'name': 'ram_event', 'code': data[0]})
        rows.append(row)
    gaps = np.diff(imu_stamps)
    maximum_gap = int(max(gaps)) if len(gaps) else None
    header.update(retained_duration_s=(receipts[-1]-receipts[0])/1e6,
                  accepted_imu_samples=len(imu_stamps), maximum_imu_gap_us=maximum_gap,
                  rejected_input_records=rejected_inputs, sha256=hashlib.sha256(raw).hexdigest(),
                  timestamp_basis='firmware receipt and producer acquisition microseconds',
                  decimation_enabled=header['imu_hz'] != 1000,
                  independent_ground_truth=False, onboard_estimator3_enabled=False)
    # Ring overwrite can remove START. A frozen snapshot is its own replay segment.
    first = {'group': 'event', 'data': {'name': 'capture_ready'}, 'received_s': 0.,
             'phase': 'ram_capture', 'device_time_us': 0}
    metadata = {'group': 'metadata', 'data': {'ram_trace': header}, 'received_s': 0.,
                'phase': 'ram_capture', 'device_time_us': 0}
    return header, [metadata, first, *rows]


def save(raw, output, diagnostics=None):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    # Preserve binary before validation so a malformed capture remains diagnosable.
    (output/'trace.bin').write_bytes(raw)
    header, rows = decode(raw)
    if diagnostics:
        header['recorder_diagnostics']=diagnostics
    write_json(output/'report.json', header)
    (output/'packets.jsonl').write_text(''.join(json.dumps(row, allow_nan=False)+'\n' for row in rows))
    return header


class Recorder:
    def __init__(self, cf):
        self.cf = cf
        expected = ('mode','imuHz','durationMs','status','session')
        if not all(key in cf.param.toc.toc.get('ramTrace', {}) for key in expected):
            raise RuntimeError('RAM recording needs the opt-in estimator_ram_trace firmware build')
        self.memory = None

    def verify_stopped(self):
        from cflib.crazyflie.log import LogConfig
        done=threading.Event()
        consecutive=[0]
        failure=[]
        def sample(tick,data,block):
            consecutive[0] = consecutive[0]+1 if all(data.get(f'motor.m{i}')==0 for i in range(1,5)) else 0
            if consecutive[0]>=2:
                done.set()
        def error(block,message):
            failure.append(str(message));done.set()
        block=LogConfig(name='ramStopped',period_in_ms=100)
        for i in range(1,5):
            block.add_variable(f'motor.m{i}','uint16_t')
        self.cf.log.add_config(block)
        block.data_received_cb.add_callback(sample)
        block.error_cb.add_callback(error)
        try:
            block.start()
            if not done.wait(2) or failure or consecutive[0]<2:
                raise RuntimeError('RAM frozen but fresh zero-motor telemetry was not confirmed; download deferred')
        finally:
            block.stop();block.delete()

    def prepare(self, hz, duration_ms):
        if hz not in (250,500,1000) or type(duration_ms) is not int or not 0 <= duration_ms <= 30000:
            raise ValueError('invalid RAM recording rate/duration')
        if self._get('status') == 1:
            raise RuntimeError('an existing RAM capture is running; freeze/download it first')
        self._set('mode',0)
        self._set('imuHz',hz)
        self._set('durationMs',duration_ms)
        self.prepared=(hz,duration_ms)
        self.prepared_session=self._get('session')

    def start_async(self, hz, duration_ms):
        # Parameters must have been prepared/read back before takeoff.
        if getattr(self,'prepared',None) != (hz,duration_ms):
            raise RuntimeError('RAM parameters were not confirmed before takeoff')
        self.expected_session=(self.prepared_session+1)&0xffffffff
        self.cf.param.set_value('ramTrace.mode','1')

    def prepare_interaction(self, hz=1000, post_release_ms=200):
        if not all(key in self.cf.param.toc.toc.get('ramTrace',{}) for key in ('trigger','postMs')):
            raise RuntimeError('RAM firmware lacks release-trigger recording support')
        self.prepare(hz,0)
        self._set('trigger',0)
        self._set('postMs',post_release_ms)

    def release_async(self):
        self.cf.param.set_value('ramTrace.trigger','1')

    def freeze_async(self):
        self.cf.param.set_value('ramTrace.mode','2')

    def _get(self, name, timeout=2):
        completed = threading.Event()
        values = []
        def callback(full_name, value):
            values.append(int(value)); completed.set()
        full = 'ramTrace.'+name
        self.cf.param.add_update_callback(group='ramTrace', name=name, cb=callback)
        try:
            self.cf.param.request_param_update(full)
            if not completed.wait(timeout):
                raise TimeoutError('no fresh RAM parameter '+full)
            return values[-1]
        finally:
            self.cf.param.remove_update_callback(group='ramTrace', name=name, cb=callback)

    def _set(self, name, value):
        self.cf.param.set_value('ramTrace.'+name, str(value))
        if self._get(name) != value:
            raise RuntimeError('RAM parameter readback mismatch: '+name)

    def start(self, hz=1000, duration_ms=600):
        if hz not in (250,500,1000) or type(duration_ms) is not int or not 1 <= duration_ms <= 30000:
            raise ValueError('RAM trace needs 250/500/1000 Hz and duration 1..30000 ms')
        # Never change either estimator selector or shadow enable from this utility.
        if int(self.cf.param.get_value('stabilizer.estimator')) != 2:
            raise RuntimeError('select default estimator 2 before RAM recording')
        if self._get('status') == 1:
            raise RuntimeError('an existing RAM capture is running; freeze/download it first')
        self._set('mode', 0)
        self._set('imuHz', hz)
        self._set('durationMs', duration_ms)
        session = self._get('session')
        self.expected_session=(session+1)&0xffffffff
        self._set('mode', 1)
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            if self._get('session') != session and self._get('status') in (1,2):
                return
            time.sleep(.01)
        raise TimeoutError('RAM recorder did not start; no flight/control mode was changed')

    def freeze(self):
        self._set('mode', 2)
        deadline=time.monotonic()+2
        while self._get('status') != 2:
            if time.monotonic() > deadline:
                raise TimeoutError('RAM recorder did not freeze')
            time.sleep(.01)

    def _read(self, memory, address, length):
        done = threading.Event()
        result = []
        def success(mem, addr, data):
            if mem.id == memory.id and addr == address:
                result.append(bytes(data)); done.set()
        def failed(mem, addr):
            if mem.id == memory.id:
                done.set()
        self.cf.mem.mem_read_cb.add_callback(success)
        self.cf.mem.mem_read_failed_cb.add_callback(failed)
        try:
            if self.cf.mem.read(memory, address, length) is False:
                raise RuntimeError('another memory read is already in progress')
            if not done.wait(max(5, length/100)):
                raise TimeoutError('RAM memory read timed out')
            if not result or len(result[0]) != length:
                raise RuntimeError('RAM memory read failed/truncated')
            return result[0]
        finally:
            self.cf.mem.mem_read_cb.remove_callback(success)
            self.cf.mem.mem_read_failed_cb.remove_callback(failed)

    def download(self):
        if self._get('status') != 2:
            raise RuntimeError('freeze RAM trace before download; recording cannot be read live')
        matches=[]
        for memory in self.cf.mem.mems:
            if memory.type != 0x18:
                continue
            try:
                header=self._read(memory,0,HEADER.size)
            except (RuntimeError, TimeoutError):
                continue
            if struct.unpack_from('<I',header)[0] == MAGIC:
                matches.append((memory,header))
        if len(matches) != 1:
            raise RuntimeError('expected exactly one frozen estimator RAM memory')
        memory, header=matches[0]
        values=HEADER.unpack(header)
        if hasattr(self,'expected_session') and values[6] != self.expected_session:
            raise RuntimeError('RAM session did not advance as requested; refusing an earlier capture')
        size=HEADER.size+values[4]*values[2]
        if values[2] != RECORD.size or values[4] > values[3] or size > memory.size:
            raise ValueError('invalid RAM memory size')
        raw=self._read(memory,0,size)
        after=self._read(memory,0,HEADER.size)
        if raw[:HEADER.size] != header or after != header:
            raise RuntimeError('RAM session changed during download; refusing mixed capture')
        self.last_download_diagnostics={}
        if 'maxHookUs' in self.cf.param.toc.toc.get('ramTrace',{}):
            self.last_download_diagnostics['max_hook_us']=self._get('maxHookUs')
        return raw


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='action',required=True)
    parse=sub.add_parser('decode');parse.add_argument('binary');parse.add_argument('--output',required=True)
    record=sub.add_parser('record');record.add_argument('--uri',default='usb://0')
    record.add_argument('--hz',type=int,choices=(250,500,1000),default=1000)
    record.add_argument('--duration-ms',type=int,default=600);record.add_argument('--output',required=True)
    download=sub.add_parser('download');download.add_argument('--uri',default='usb://0')
    download.add_argument('--output',required=True)
    args=parser.parse_args(argv)
    if args.action=='decode':
        report=save(Path(args.binary).read_bytes(),args.output)
    else:
        if args.action=='record' and input('Bench capture only. Remove props. Type PROPS OFF: ').strip() != 'PROPS OFF':
            raise ValueError('props-off confirmation missing')
        import cflib.crtp
        from cflib.crazyflie import Crazyflie
        from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
        cflib.crtp.init_drivers()
        with SyncCrazyflie(args.uri,cf=Crazyflie(rw_cache=None)) as link:
            recorder=Recorder(link.cf)
            if args.action=='record':
                recorder.start(args.hz,args.duration_ms)
                try:
                    time.sleep(args.duration_ms/1000+.05)
                finally:
                    recorder.freeze()
            raw=recorder.download()
            report=save(raw,args.output,recorder.last_download_diagnostics)
    print(json.dumps(report,indent=2))

if __name__=='__main__':
    main()
