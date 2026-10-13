"""Bounded contact-free capture for offline estimator-3 replay; estimator 2 flies.

The fixture candidate is never uploaded to firmware. Only default estimator 2
runs onboard; Vicon forwards position only and estimator 3 runs offline.
"""

from contextlib import contextmanager
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import signal
import threading
import time

import numpy as np
import yaml

from Interaction.estimator_imu_calibration import G, write_json
from Interaction.imu_logging import fls_log_config


@contextmanager
def flight_signals():
    """Let SSH hangup/termination reach the same descent/finally path as Ctrl-C."""
    previous = {}
    def interrupted(signum, frame):
        raise KeyboardInterrupt(f'flight interrupted by signal {signum}')
    if threading.current_thread() is threading.main_thread():
        for name in ('SIGINT', 'SIGTERM', 'SIGHUP'):
            signum = getattr(signal, name, None)
            if signum is not None:
                previous[signum] = signal.signal(signum, interrupted)
    try:
        yield
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)


def vector(value, size, label):
    out = np.asarray(value, dtype=float)
    if out.shape != (size,) or not np.isfinite(out).all():
        raise ValueError(f'{label} must contain {size} finite numbers')
    return out


def quaternion(value):
    q = vector(value, 4, 'quaternion wxyz')
    norm = np.linalg.norm(q)
    if not .9 <= norm <= 1.1:
        raise ValueError('quaternion must have unit norm')
    return q / norm


def multiply(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return np.array([w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
                     w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v])


def rotation(q):
    w, x, y, z = quaternion(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def euler(q):
    r = rotation(q)
    return np.rad2deg([np.arctan2(r[2, 1], r[2, 2]),
                      np.arcsin(np.clip(-r[2, 0], -1, 1)),
                      np.arctan2(r[1, 0], r[0, 0])])


def builtin_flight_config(initial_position):
    """Fixed calibration motion; no mission/detector/release YAML is consulted."""
    from Interaction import config as defaults
    ground=vector(initial_position,3,'manifest init_pos')
    center=ground+np.array([0.,0.,.6])
    return {'builtin_profile':'imu_validation_v1',
            'mocap_host':defaults.MOCAP_HOST_NAME,'tracking_mode':'pointcloud',
            'initial_position_m':ground.tolist(),'attitude_reference':'default_estimator',
            'center_m':center.tolist(),
            'boundary_m':[[float(ground[0]-.6),float(ground[0]+.6)],
                          [float(ground[1]-.6),float(ground[1]+.6)],
                          [float(ground[2]-.05),float(ground[2]+1.)]],
            'minimum_voltage':float(defaults.MIN_LIHV_VOLT)*2}


def configure_builtin_capture(config, parameters):
    if config.get('builtin_profile')!='imu_validation_v1':
        return None
    available=all(key in parameters.get('ramTrace',{})
                  for key in ('mode','imuHz','durationMs','status','session'))
    if available:
        config['ram_trace']={'phase':'+Y_out','offset_s':1.,'hz':1000,'duration_ms':600}
    return available


def anchor_builtin_flight(config, position):
    if config.get('builtin_profile')=='imu_validation_v1':
        actual=builtin_flight_config(position)
        config['center_m']=actual['center_m']
        config['boundary_m']=actual['boundary_m']
        config['measured_ground_m']=list(position)


def load_flight_config(path):
    try:
        config = yaml.safe_load(Path(path).read_text())
    except yaml.YAMLError as error:
        raise ValueError('invalid flight YAML: '+str(error)) from error
    if not isinstance(config, dict):
        raise ValueError('flight config must be a mapping')
    if 'builtin_profile' in config:
        if config.get('builtin_profile')!='imu_validation_v1' or set(config)!={'builtin_profile','initial_position_m'}:
            raise ValueError('invalid built-in calibration profile')
        config=builtin_flight_config(config['initial_position_m'])
    required = {'mocap_host', 'center_m', 'boundary_m', 'minimum_voltage'}
    optional = {'tracking_mode', 'initial_position_m', 'rigidbody', 'attitude_reference',
                'body_to_rigidbody_wxyz', 'orientation_reference_note', 'ram_trace','builtin_profile'}
    if not required.issubset(config) or set(config)-required-optional:
        raise ValueError('flight config needs mocap_host, center_m, boundary_m, minimum_voltage and tracking settings')
    mode = config.get('tracking_mode', 'rigidbody' if config.get('rigidbody') else 'pointcloud')
    if mode not in ('rigidbody', 'pointcloud'):
        raise ValueError('tracking_mode must be rigidbody or pointcloud')
    if not isinstance(config['mocap_host'], str) or not config['mocap_host'].strip():
        raise ValueError('mocap_host cannot be empty')
    if mode == 'pointcloud':
        vector(config.get('initial_position_m'), 3, 'initial_position_m')
    elif not isinstance(config.get('rigidbody'), str) or not config['rigidbody'].strip():
        raise ValueError('rigidbody name cannot be empty')
    if config.get('attitude_reference', 'default_estimator') != 'default_estimator':
        raise ValueError('this protocol uses default_estimator as the flight reference')
    center = vector(config['center_m'], 3, 'center_m')
    bounds = np.asarray(config['boundary_m'], dtype=float)
    if bounds.shape != (3, 2) or not np.isfinite(bounds).all() or np.any(bounds[:, 0] >= bounds[:, 1]):
        raise ValueError('boundary_m must be [[x_min,x_max],[y_min,y_max],[z_min,z_max]]')
    # All target points have 20 cm tracking/error margin; the ground level is
    # checked separately against Vicon before arming.
    margin = np.array([.4, .4, .2])
    if np.any(center-margin < bounds[:, 0]) or np.any(center+margin > bounds[:, 1]):
        raise ValueError('flight center needs 0.4m XY and 0.2m Z boundary clearance')
    voltage = config['minimum_voltage']
    if isinstance(voltage, bool) or not isinstance(voltage, (int, float)) or not 3. <= voltage <= 9.:
        raise ValueError('minimum_voltage must be explicitly set between 3 and 9 V for this aircraft')
    if 'ram_trace' in config:
        from Interaction.estimator_ram_trace import validate_flight_trace
        validate_flight_trace(config['ram_trace'])
    return config


def verify_capture_continuity(dataset, snapshot, *, clock_reference=None):
    before = dataset.get('capture', {}).get('firmware_parameters_after', {}).get('imu_sensors')
    metadata = snapshot.get('metadata', {}).get('data', {})
    after = metadata.get('firmware_parameters_before', {}).get('imu_sensors')
    if before is None or after != before:
        raise RuntimeError('IMU configuration unavailable or changed since fixture capture')
    last_pose = clock_reference if clock_reference is not None else (dataset.get('rotations') or dataset['poses'])[-1]
    host_gap = snapshot['imu']['received_s']-last_pose['host_receipt_monotonic_ns'][-1]/1e9
    device_gap = ((snapshot['imu']['cf_log_tick_ms_mod24']-last_pose['cf_log_tick_ms_mod24'][-1]) % (1 << 24))/1000.
    if host_gap < 0 or abs(device_gap-host_gap) > 2:
        raise RuntimeError('firmware clock continuity lost; repeat static fit without power cycling')


def trajectory(center, ground):
    """Fifth-order position ramps; four symmetric 20 cm excursions, no push."""
    center, ground = np.asarray(center, float), np.asarray(ground, float)
    phases = [('takeoff', ground, center, 4.), ('hover_start', center, center, 5.)]
    for axis, sign, name in ((0, 1, '+X'), (0, -1, '-X'), (1, 1, '+Y'), (1, -1, '-Y')):
        target = center.copy()
        target[axis] += sign * .2
        phases += [(f'{name}_out', center, target, 3.),
                   (f'{name}_hold', target, target, 3.),
                   (f'{name}_return', target, center, 3.),
                   (f'{name}_center', center, center, 3.)]
    phases += [('hover_end', center, center, 5.), ('land', center, ground, 4.)]
    return phases


class TelemetryStaleError(RuntimeError):
    def __init__(self, group, age_s):
        self.group, self.age_s = group, age_s
        detail = 'missing' if age_s is None else f'age={age_s*1000:.1f}ms'
        super().__init__(f'{group} telemetry missing or stale ({detail})')


def check_snapshot(snapshot, config, *, flying, ground_z):
    now = snapshot['now']
    for group in ('vicon', 'imu', 'kf', 'state', 'health'):
        row = snapshot.get(group)
        age_s = None if not row else now-row['received_s']
        if age_s is None or not 0 <= age_s <= (.3 if group == 'health' else .1):
            raise TelemetryStaleError(group, age_s)
    state, pose = snapshot['state']['data'], snapshot['vicon']['data']
    measured = vector([state[f'stateEstimate.{a}'] for a in 'xyz'], 3, 'onboard position')
    actual = vector(pose['position_m'], 3, 'Vicon position')
    bounds = np.asarray(config['boundary_m'])
    low = bounds[:, 0].copy()
    low[2] = min(low[2], ground_z-.05)  # takeoff/landing must include the measured floor
    if np.any(actual < low) or np.any(actual > bounds[:, 1]):
        raise RuntimeError('Vicon position exceeded validation boundary')
    if np.linalg.norm(actual-measured) > .2:
        raise RuntimeError('ordinary estimator and Vicon position disagree by >0.2m')
    if max(abs(euler([snapshot['kf']['data'][f'kalman.q{i}'] for i in range(4)])[:2])) > 20:
        raise RuntimeError('ordinary estimator tilt exceeds 20 degrees')
    health = snapshot['health']['data']
    if not math.isfinite(health['pm.vbat']) or health['pm.vbat'] < config['minimum_voltage']:
        raise RuntimeError('battery below configured flight minimum')
    if not flying and any(health[f'motor.m{i}'] != 0 for i in range(1, 5)):
        raise RuntimeError('motor output nonzero before arming')


LOGS = {
    'imu': (10, {f'{s}.{a}': 'float' for s in ('acc', 'gyro') for a in 'xyz'}),
    'kf': (20, {f'kalman.q{i}': 'float' for i in range(4)}),
    'state': (20, {f'stateEstimate.{a}': 'float' for a in ('x', 'y', 'z', 'vx', 'vy', 'vz')}),
    'health': (100, {'pm.vbat': 'float', **{f'motor.m{i}': 'uint16_t' for i in range(1, 5)},
                     'supervisor.info': 'uint16_t'}),
}
PARAMETERS = {'stabilizer.estimator': 2, 'stabilizer.controller': 1}
OPTIONAL_DISABLED = ('hlCommander.pRelAuto', 'kalmanPRel.enable', 'kalmanPRel.scEnable')

class FlightConnection:
    """Small dedicated cflib adapter. Never creates a normal interaction mission."""
    def __init__(self, cf, config, journal):
        self.cf, self.config, self.journal = cf, config, journal
        self.lock = threading.RLock()
        self.latest = {}
        self.failure = None
        self.phase = 'preflight'

    def record(self, group, data, tick=None):
        row = {'group': group, 'data': data, 'cf_log_tick_ms_mod24': tick,
               'received_s': time.monotonic(), 'phase': self.phase}
        with self.lock:
            self.journal.write(json.dumps(row, allow_nan=False) + '\n')
            self.journal.flush()
            self.latest[group] = row

    def snapshot(self, *, include_metadata=True):
        with self.lock:
            if self.failure:
                raise RuntimeError(self.failure)
            # Rows are replaced atomically, never edited after publication.
            # Copy outside the writer lock so callbacks can publish new data.
            latest = {k:v for k,v in self.latest.items() if include_metadata or k!='metadata'}
        if int(self.cf.param.get_value('stabilizer.estimator')) != 2:
            raise RuntimeError('control estimator changed during validation')
        return {**deepcopy(latest), 'now': time.monotonic()}

    def command(self, position, yaw_deg):
        self.cf.commander.send_position_setpoint(*[float(x) for x in position], float(yaw_deg))
        self.record('command', {'position_m': list(map(float, position)), 'yaw_deg': float(yaw_deg),
                                'estimator': 2})

    def arm(self):
        self.record('event', {'name': 'arm_requested'})
        self.cf.platform.send_arming_request(True)

    def stop(self):
        try:
            self.cf.commander.send_stop_setpoint()
        finally:
            self.cf.platform.send_arming_request(False)
        self.record('event', {'name': 'stop_disarm_requested'})
        began = time.monotonic()
        while time.monotonic()-began < 2:
            with self.lock:
                health = self.latest.get('health', {})
                if health.get('received_s', -1) >= began and all(
                        health.get('data', {}).get(f'motor.m{i}') == 0 for i in range(1, 5)):
                    self.record('event', {'name': 'zero_motor_output_confirmed'})
                    return
            time.sleep(.02)
        raise RuntimeError('fresh zero-motor telemetry not confirmed after stop/disarm')


@contextmanager
def open_flight(uri, config, journal):
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.log import LogConfig
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from mocap import Mocap
    from Interaction.firmware_parameter_confirmation import confirm_firmware_mode_parameters

    def write_parameters(cf, values):
        numeric = {key: float(value) if isinstance(value, str) else value for key, value in values.items()}
        for name, value in numeric.items():
            cf.param.set_value(name, str(int(value)) if float(value).is_integer() else str(value))
        confirm_firmware_mode_parameters(cf.param, expected=numeric)

    cflib.crtp.init_drivers()
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache=None)) as scf:
        cf = scf.cf
        deadline = time.monotonic()+15
        while not cf.param.is_updated:
            if time.monotonic() > deadline:
                raise TimeoutError('flight firmware parameters unavailable')
            time.sleep(.05)
        missing = [f for _, fields in LOGS.values() for f in fields
                   if f.split('.')[1] not in cf.log.toc.toc.get(f.split('.')[0], {})]
        disabled = {key: 0 for key in OPTIONAL_DISABLED if key.split('.')[1] in cf.param.toc.toc.get(key.split('.')[0], {})}
        expected = {**PARAMETERS, **disabled, 'kalman.resetEstimation': 0}
        missing += [f for f in expected if f.split('.')[1] not in cf.param.toc.toc.get(f.split('.')[0], {})]
        if missing:
            raise RuntimeError('default-estimator flight capture requires firmware interfaces: '+', '.join(missing))
        saved = {key: cf.param.values[key.split('.')[0]][key.split('.')[1]] for key in expected}
        saved['stabilizer.estimator'] = '2'
        if 'kalmanPRel.enable' in saved:
            saved['kalmanPRel.enable'] = '0'
        link = FlightConnection(cf, config, journal)
        available=configure_builtin_capture(config,cf.param.toc.toc)
        if available is not None:
            print('[estimator validation] capture: '+('1000 Hz RAM + 100 Hz logs' if available else
                  '100 Hz logs (RAM recorder unavailable)'),flush=True)
            link.record('event',{'name':'builtin_capture_selected','ram_available':available,
                                  'sampling':'ram_1000hz_plus_log_100hz' if available else 'log_100hz'})
        if 'ram_trace' in config:
            from Interaction.estimator_ram_trace import Recorder
            link.ram_recorder = Recorder(cf)
            link.ram_started = False
        link.record('metadata', {'firmware_parameters_before': deepcopy(cf.param.values),
                                'log_periods_ms': {k: v[0] for k, v in LOGS.items()},
                                'orientation_forwarded': False, 'candidate_applied': False,
                                'onboard_estimator3_enabled': False,
                                'timestamp_basis': 'host receipt; CF log ticks also saved; not camera capture time'})
        blocks, mocap = [], None
        closing = threading.Event()

        def on_data(tick, data, block):
            if closing.is_set():
                return
            try:
                link.record(block.name, dict(data), tick)
            except Exception as error:
                link.failure = str(error)

        def on_error(block, error):
            link.failure = f'{block.name}: {error}'

        def on_pose(frame):
            if closing.is_set():
                return
            try:
                pos = vector(frame['tvec'], 3, 'Vicon position').tolist()
                received = time.monotonic()
                cf.extpos.send_extpos(*pos)
                link.record('vicon', {'position_m': pos, 'frame_id': frame['frame_id'],
                                      'host_wall_time': frame['time'], 'position_forwarded_s': received})
            except Exception as error:
                link.failure = 'Vicon forwarding: '+str(error)

        try:
            # Fresh ACK/readback; use the same confirmation helper as S-curve.
            write_parameters(cf, {**PARAMETERS, **disabled})
            for name, (period, fields) in LOGS.items():
                block = fls_log_config(name, period)
                for field, kind in fields.items():
                    block.add_variable(field, kind)
                cf.log.add_config(block)
                blocks.append(block)
                block.data_received_cb.add_callback(on_data)
                block.error_cb.add_callback(on_error)
                block.start()
            mode = config.get('tracking_mode', 'rigidbody' if config.get('rigidbody') else 'pointcloud')
            mocap = Mocap(host_name=config['mocap_host'], mode=mode)
            mocap.daemon = True
            if mode == 'rigidbody':
                mocap.subscribe_object(config['rigidbody'], on_pose)
            else:
                mocap.subscribe_point(config['initial_position_m'], on_pose, name='imu_validation')
            mocap.start()
            # Bound every wait. No arming until ordinary position convergence
            # is observed. No ESKF shadow is enabled during this capture.
            deadline = time.monotonic()+10
            while not all(k in link.snapshot() for k in ('vicon', 'imu', 'kf', 'state', 'health')):
                if time.monotonic() > deadline:
                    raise TimeoutError('Vicon/firmware streams not ready before flight')
                time.sleep(.02)
            pose = link.snapshot()['vicon']['data']
            # The manifest selects the marker; measured Vicon anchors motion.
            anchor_builtin_flight(config,pose['position_m'])
            yaw = math.radians(euler([link.snapshot()['kf']['data'][f'kalman.q{i}'] for i in range(4)])[2])
            # Default estimator provides the requested relative attitude reference.
            initial = {f'kalman.initial{a.upper()}': float(pose['position_m'][i]) for i, a in enumerate('xyz')}
            initial['kalman.initialYaw'] = yaw
            for name in initial:
                if name.split('.')[1] not in cf.param.toc.toc.get('kalman', {}):
                    raise RuntimeError('missing firmware initialization parameter '+name)
                saved[name] = cf.param.values['kalman'][name.split('.')[1]]
            write_parameters(cf, initial)
            # resetEstimation is a self-clearing request, not a stable mode.
            cf.param.set_value('kalman.resetEstimation', '1')
            time.sleep(2)
            confirm_firmware_mode_parameters(cf.param, expected={'kalman.resetEstimation': 0})
            if 'ram_trace' in config:
                trace_config=config['ram_trace']
                link.ram_recorder.prepare(trace_config['hz'],trace_config['duration_ms'])
            link.record('event', {'name': 'capture_ready', 'control_estimator': 2,
                                  'onboard_estimator3_enabled': False})
            yield link
        finally:
            # The protocol lands/stops before exiting this scope. Restore all
            # touched runtime parameters only with no command authority.
            closing.set()
            if getattr(link, 'ram_started', False):
                # Also freeze on a failed/aborted capture. Never erase its RAM.
                cf.param.set_value('ramTrace.mode','2')
            if mocap:
                mocap.running = False
                mocap.join(timeout=1)
            for block in blocks:
                try:
                    block.stop()
                    block.delete()
                except Exception:
                    pass
            try:
                write_parameters(cf, saved)
                link.record('event', {'name': 'parameters_restored'})
            except Exception as error:
                link.record('event', {'name': 'parameter_restore_failed', 'error': str(error)})
                raise


def run_protocol(link, config, *, clock=time.monotonic, sleep=time.sleep):
    """Check readiness before arm; on any airborne error request bounded descent."""
    center = np.asarray(config['center_m'], float)
    deadline = clock()+8
    while True:
        try:
            snap = link.snapshot()
            ground = vector(snap['vicon']['data']['position_m'], 3, 'ground position')
            check_snapshot(snap, config, flying=False, ground_z=ground[2])
            break
        except (KeyError, RuntimeError):
            if clock() > deadline:
                raise RuntimeError('preflight did not converge; no arming request sent')
            sleep(.02)
    if np.linalg.norm(ground[:2]-center[:2]) > .1 or not .2 <= center[2]-ground[2] <= 1.2:
        raise RuntimeError('place aircraft under flight center; height must be 0.2–1.2m above the measured floor')
    original_ground = ground.copy()
    ground[2] += .02
    yaw = float(euler([snap['kf']['data'][f'kalman.q{i}'] for i in range(4)])[2])
    armed = False
    last = ground.copy()
    try:
        armed = True  # an uncertain arming request still needs stop/disarm
        link.arm()
        for phase, start, end, duration in trajectory(center, ground):
            link.phase = phase
            print(f'[estimator validation] {phase}', flush=True)
            began = clock()
            while True:
                elapsed = clock()-began
                u = min(1., elapsed/duration)
                fraction = 10*u**3-15*u**4+6*u**5
                last = start+(end-start)*fraction
                snap = link.snapshot()
                check_snapshot(snap, config, flying=True, ground_z=original_ground[2])
                if np.linalg.norm(np.asarray(snap['vicon']['data']['position_m'])-last) > .25:
                    raise RuntimeError('aircraft deviated from validation target by >0.25m')
                link.command(last, yaw)
                trace_config = config.get('ram_trace')
                if (trace_config and not link.ram_started and phase == trace_config['phase']
                        and elapsed >= trace_config.get('offset_s', 0)):
                    # Only write requests here; no blocking readback/download during flight.
                    link.ram_recorder.start_async(trace_config['hz'], trace_config['duration_ms'])
                    link.ram_started = True
                    link.record('event', {'name': 'ram_record_requested', 'phase': phase})
                if u >= 1:
                    break
                sleep(.02)
        # Do not infer touchdown from the scheduled end time alone.
        deadline = clock()+2
        while True:
            snap = link.snapshot()
            check_snapshot(snap, config, flying=True, ground_z=original_ground[2])
            if abs(snap['vicon']['data']['position_m'][2]-ground[2]) <= .05:
                break
            if clock() > deadline:
                raise RuntimeError('landing target not reached before stop')
            link.command(ground, yaw)
            sleep(.02)
        return {'capture_completed': True, 'control_estimator': 2,
                'candidate_applied': False, 'active_estimator3_tested': False}
    except BaseException:
        if armed:
            link.phase = 'abort_land'
            # Onboard position, if fresh, is preferable to a stale Vicon point.
            try:
                snap = link.snapshot()
                if snap['now']-snap['state']['received_s'] <= .1:
                    last = np.array([snap['state']['data'][f'stateEstimate.{a}'] for a in 'xyz'])
            except Exception:
                pass
            descent_start = last.copy()
            began = clock()
            try:
                while clock()-began < 4:
                    u = min(1., (clock()-began)/4)
                    target = descent_start.copy()
                    target[2] = (1-u)*descent_start[2]+u*ground[2]
                    link.command(target, yaw)
                    sleep(.02)
            except BaseException:
                pass  # second interrupt/link loss must not postpone stop
        raise
    finally:
        if armed:
            link.stop()


def summarize_capture(path, candidate):
    rows = [json.loads(line) for line in Path(path).read_text().splitlines()]
    return {'attitude_reference': 'default_estimator', 'independent_ground_truth': False,
            'onboard_estimator3_enabled': False, 'packets': len(rows),
            'note': 'Raw capture only. Run actual C estimator-3 replay on the analysis host after download; '
                    'default estimator is a relative reference and shares the same IMU.'}


def run_validation_flight(uri, candidate_path, dataset_path, config_path, output, *,
                          prompt=input, connection=open_flight, initial_position=None):
    config = load_flight_config(config_path) if config_path is not None else builtin_flight_config(initial_position)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    candidate = json.loads(Path(candidate_path).read_text())
    raw = Path(dataset_path).read_bytes()
    if candidate.get('fit_passed') is not True or candidate.get('dataset_sha256') != hashlib.sha256(raw).hexdigest():
        raise ValueError('flight requires the successful static IMU fit for this exact dataset')
    dataset = json.loads(raw)
    result = {'capture_completed': False, 'flight_started': False, 'control_estimator': 2,
              'candidate_applied': False, 'calibrated_estimator3_validated': False,
              'candidate_sha256': hashlib.sha256(Path(candidate_path).read_bytes()).hexdigest(),
              'config': config, 'drone_id': candidate['drone_id'], 'firmware_id': candidate['firmware_id']}
    write_json(output / 'report.json', result)
    write_json(output / 'config.json', config)
    try:
        print('Fixture fit saved. Keep aircraft powered. Refit props and place it under the configured center.')
        print('Clear flight area; no hand contact. Only estimator 2 runs; estimator 3 will run offline after download.')
        if config.get('builtin_profile'):
            print('Built-in flight: hover 0.6m above measured start, move ±0.2m in X/Y, then land. No interaction required.')
        prompt('Press ENTER to start automatic takeoff, ±X/±Y sampling and landing (Ctrl+C to cancel): ')
        with flight_signals(), (output / 'packets.jsonl').open('w') as journal, connection(uri, config, journal) as link:
            deadline = time.monotonic()+3
            while 'imu' not in link.snapshot():
                if time.monotonic() > deadline:
                    raise TimeoutError('no IMU telemetry for same-boot check')
                time.sleep(.02)
            snap = link.snapshot()
            verify_capture_continuity(dataset, snap)
            write_json(output/'config.json',config)
            result['recording_mode']='ram_1000hz_plus_log_100hz' if config.get('ram_trace') else 'log_100hz'
            # run_protocol performs the last readiness checks before the arm call.
            result.update(run_protocol(link, config))
            result['flight_started'] = True
            if 'ram_trace' in config:
                # run_protocol has stopped/disarmed and confirmed zero motor output.
                if not link.ram_started:
                    raise RuntimeError('RAM capture phase was not reached')
                from Interaction.estimator_ram_trace import save
                link.ram_recorder.freeze()
                raw=link.ram_recorder.download()
                result['ram_trace'] = save(raw, output/'ram_trace',link.ram_recorder.last_download_diagnostics)
        result['diagnostics'] = summarize_capture(output / 'packets.jsonl', candidate)
    except (Exception, KeyboardInterrupt) as error:
        result['capture_completed'] = False
        result['error'] = str(error) or type(error).__name__
    finally:
        packet_path = output / 'packets.jsonl'
        if packet_path.exists():
            result['packets_sha256'] = hashlib.sha256(packet_path.read_bytes()).hexdigest()
            result['flight_started'] = any('"name": "arm_requested"' in line for line in packet_path.read_text().splitlines())
        write_json(output / 'report.json', result)
        (output / 'report.md').write_text(
            '# Estimator 3 validation capture\n\n'
            f'Capture completed: {result["capture_completed"]}\n\n'
            f'Flight started: {result["flight_started"]}\n\n'
            'Control estimator: 2. Candidate applied: False. Calibrated estimator 3 validated: False.\n\n'
            'IMU, Vicon positions and default-estimator attitude/state are recorded. '
            'Estimator 3 is disabled onboard and runs only in offline replay. Default estimator is a relative reference, not independent ground truth.\n\n'
            + result.get('error', '') + '\n')
    print(f'Flight capture {"COMPLETED" if result["capture_completed"] else "STOPPED"}: {output}')
    return result


def run_standard_validation(uri, candidate_path, dataset_path, config_path, output, *,
                            prompt=input, initial_position=None):
    source = Path(candidate_path).resolve().parent.parent
    if Path(dataset_path).resolve() != source/'dataset.json':
        raise ValueError('standard validation needs dataset and candidate from one saved session')
    fit = json.loads(Path(candidate_path).read_text())
    tokens = ['--uri', uri, '--drone-id', fit['drone_id'], '--calibration',str(source),
              '--output',str(output)]
    if config_path is not None:
        tokens += ['--flight-config',str(config_path)]
    if initial_position is not None:
        tokens += ['--initial-position',*map(str,initial_position)]
    code = main(tokens, prompt=prompt)
    report_path = Path(output)/'report.json'
    report = json.loads(report_path.read_text()) if report_path.exists() else {'capture_completed':False}
    if code:
        report['capture_completed'] = False
    return report


def main(argv=None, *, prompt=input):
    import argparse
    import subprocess
    import sys
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--uri', default='usb://0')
    parser.add_argument('--drone-id', required=True)
    parser.add_argument('--calibration', type=Path, required=True,
                        help='saved session containing dataset.json and fit/candidate.json')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--flight-config', type=Path)
    parser.add_argument('--initial-position', nargs=3, type=float)
    args = parser.parse_args(argv)
    candidate = args.calibration / 'fit/candidate.json'
    dataset = args.calibration / 'dataset.json'
    try:
        if json.loads(candidate.read_text()).get('drone_id') != args.drone_id:
            raise ValueError('saved calibration belongs to another aircraft')
        raw = dataset.read_bytes()
        fit = json.loads(candidate.read_text())
        if fit.get('fit_passed') is not True or fit.get('dataset_sha256') != hashlib.sha256(raw).hexdigest():
            raise ValueError('flight requires a successful matching static IMU fit')
        config = load_flight_config(args.flight_config) if args.flight_config is not None else builtin_flight_config(args.initial_position)
        if config.get('ram_trace'):
            raise ValueError('standard-controller IMU validation currently uses ordinary logs; remove ram_trace from override')
        if args.uri != 'usb://0' and not args.uri.startswith('radio://'):
            raise ValueError('standard controller supports usb://0 or radio:// URI')
        print('Standard controller localization, PID setup, takeoff and landing will be used.')
        print('Keep aircraft powered. Install props, place marker at starting position, and clear flight area.')
        # The normal Controller owns the single pre-arm Enter prompt.
        command = [sys.executable, '-u', 'controller.py', '--calibrate', '--log', '--vicon',
                   '--vicon-mode', config.get('tracking_mode','rigidbody' if config.get('rigidbody') else 'pointcloud'), '--drone-id', args.drone_id,
                   '--init-pos', *map(str, config.get('initial_position_m',config['center_m'])),
                   '--takeoff-altitude', str(config['center_m'][2]),
                   '--smooth-controller-rate', '100', '--cf-log-period', '10',
                   '--imu-validation-session', str(args.calibration.resolve()),
                   '--imu-validation-output', str(args.output.resolve()),
                   '--tag', 'imu_validation_'+args.output.parent.name]
        if args.flight_config is not None:
            command += ['--imu-validation-config',str(args.flight_config.resolve())]
        if config.get('rigidbody'):
            command += ['--obj-name',config['rigidbody']]
        if args.uri.startswith('radio://'):
            command += ['--radio', args.uri]
        completed = subprocess.run(command, cwd=Path(__file__).resolve().parents[1])
        report_path = args.output/'report.json'
        report = json.loads(report_path.read_text()) if report_path.exists() else {}
        return 0 if completed.returncode == 0 and report.get('capture_completed') is True else 1
    except (ValueError, OSError, KeyboardInterrupt) as error:
        print(f'Flight capture stopped: {str(error) or type(error).__name__}')
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
