"""IMU capture task inside the normal Controller lifecycle; never selects ESKF."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from Interaction.estimator_imu_calibration import G, validate_pose, write_json
from Interaction.imu_logging import fls_log_config
from Interaction.estimator_validation_flight import (
    FlightConnection, LOGS, OPTIONAL_DISABLED, anchor_builtin_flight,
    builtin_flight_config, load_flight_config, check_snapshot, euler, trajectory, verify_capture_continuity,
    TelemetryStaleError,
)
from Interaction.firmware_parameter_confirmation import confirm_firmware_mode_parameters
from Interaction.estimator_hover_initialization import hover_window, SETTLE_S, DURATION_S


class ControllerValidationCapture:
    def __init__(self, owner):
        self.owner = owner
        source = Path(owner.args.imu_validation_session)
        self.output = Path(owner.args.imu_validation_output)
        self.output.mkdir(parents=True, exist_ok=False)
        raw = (source/'dataset.json').read_bytes()
        candidate_raw = (source/'fit/candidate.json').read_bytes()
        self.candidate = json.loads(candidate_raw)
        self.dataset = json.loads(raw)
        if (self.candidate.get('fit_passed') is not True
                or self.candidate.get('drone_id') != owner.args.drone_id
                or self.candidate.get('dataset_sha256') != hashlib.sha256(raw).hexdigest()):
            raise ValueError('standard flight requires a matching successful IMU candidate')
        override = getattr(owner.args,'imu_validation_config',None)
        self.config = load_flight_config(override) if override else builtin_flight_config(owner.args.init_pos)
        self.report = dict(capture_completed=False, flight_started=False,
            control_estimator=2, candidate_applied=False, calibrated_estimator3_validated=False,
            candidate_sha256=hashlib.sha256(candidate_raw).hexdigest(),
            drone_id=self.candidate['drone_id'], firmware_id=self.candidate['firmware_id'],
            flight_lifecycle='controller_standard', recording_mode='log_100hz', config=self.config)
        self.blocks = []
        self.finished = False
        self.motion_completed = False
        self.clock_reference = None
        self.journal = (self.output/'packets.jsonl').open('w')
        self.link = FlightConnection(owner.cf, self.config, self.journal)
        self.save()

    def save(self):
        write_json(self.output/'report.json', self.report)
        write_json(self.output/'config.json', self.config)
        (self.output/'report.md').write_text(
            '# Standard-controller IMU validation capture\n\n'
            + json.dumps(self.report, indent=2) + '\n')

    def on_pose(self, frame):
        with self.link.lock:
            if not self.finished:
                self.link.record('vicon', dict(position_m=list(frame['tvec']),
                    frame_id=frame.get('frame_id'), host_wall_time=frame.get('time'),
                    position_forwarded_s=time.monotonic()))

    def prepare(self):
        cf = self.owner.cf
        disabled = {key:0 for key in OPTIONAL_DISABLED
                    if key.split('.')[1] in cf.param.toc.toc.get(key.split('.')[0], {})}
        for key in disabled:
            cf.param.set_value(key, '0')
        confirm_firmware_mode_parameters(cf.param, expected={
            'stabilizer.estimator':2, 'stabilizer.controller':1, **disabled})
        self.link.record('metadata', dict(firmware_parameters_before=deepcopy(cf.param.values),
            onboard_estimator3_enabled=False, candidate_applied=False, orientation_forwarded=False,
            log_periods_ms={k:v[0] for k,v in LOGS.items()},
            crtp_log_period_unit_ms=1, cflib_period_scale=10))
        for group, (period, fields) in LOGS.items():
            block = fls_log_config('imuval_'+group, period)
            for field, kind in fields.items():
                block.add_variable(field, kind)
            cf.log.add_config(block)
            self.blocks.append(block)
            def received(tick, data, _block, group=group):
                with self.link.lock:
                    if not self.finished:
                        try:
                            self.link.record(group, dict(data), tick)
                        except Exception as error:
                            self.link.failure = str(error)
            def failed(_block, error):
                self.link.failure = str(error)
            block.data_received_cb.add_callback(received)
            block.error_cb.add_callback(failed)
            block.start()
        deadline = time.monotonic()+10
        while True:
            snap = self.link.snapshot()
            if all(k in snap for k in ('vicon','imu','kf','state','health')):
                anchor_builtin_flight(self.config, snap['vicon']['data']['position_m'])
                # Standard takeoff accepts WORLD altitude. Derive it from the
                # measured floor, after Dispatcher selection, not the old seed.
                if self.config.get('builtin_profile') == 'imu_validation_v1':
                    self.owner.args.takeoff_altitude = self.config['center_m'][2]
                else:
                    self.config['center_m'][2] = self.owner.args.takeoff_altitude
                self.config['boundary_m'][2][1] = max(self.config['boundary_m'][2][1], self.owner.args.takeoff_altitude+.4)
                check_snapshot(snap, self.config, flying=False, ground_z=self.owner.init_coord[2])
                self.check_supervisor(snap, require_armable=True)
                self.refresh_gyro_zero()
                snap = self.link.snapshot()
                verify_capture_continuity(self.dataset, snap, clock_reference=self.clock_reference)
                self.link.record('event', {'name':'capture_ready', 'control_estimator':2,
                    'candidate_applied':False, 'onboard_estimator3_enabled':False})
                self.save()
                break
            if time.monotonic() > deadline:
                raise TimeoutError('standard validation telemetry not ready: '+', '.join(
                    k for k in ('vicon','imu','kf','state','health') if k not in snap))
            self.owner._safe_sleep(.02)

    def arm_requested(self):
        snap = self.link.snapshot()
        check_snapshot(snap, self.config, flying=False, ground_z=self.owner.init_coord[2])
        self.check_supervisor(snap, require_armable=True)
        verify_capture_continuity(self.dataset, snap, clock_reference=self.clock_reference)
        self.report['flight_started'] = True
        self.link.phase = 'takeoff'
        self.link.record('event', {'name':'arm_requested'})
        self.save()

    def check_supervisor(self, snap, *, require_armable=False):
        info = int(snap['health']['data']['supervisor.info'])
        faults = [name for bit, name in ((5,'tumbled'), (6,'LOCKED; restart flight controller'),
                  (7,'crashed'), (11,'deck fault')) if info & (1 << bit)]
        if faults or (require_armable and not info & 1):
            detail = ', '.join(faults) if faults else 'not ready to arm'
            self.report['error'] = f'firmware supervisor prohibits validation: {detail} (info={info})'
            self.save()
            raise RuntimeError(self.report['error'])
        return info

    def refresh_gyro_zero(self):
        """Keep the saved accelerometer fit; measure this boot's gyro zero on the floor."""
        print('[estimator validation] keep still: refreshing gyro zero before arming', flush=True)
        self.owner._safe_sleep(2.)
        began = time.monotonic(); samples = []; ticks = []; receipts = []
        while time.monotonic()-began < 3.1:
            snap = self.link.snapshot()
            check_snapshot(snap, self.config, flying=False, ground_z=self.owner.init_coord[2])
            self.check_supervisor(snap, require_armable=True)
            row = snap['imu']; tick = row['cf_log_tick_ms_mod24']
            if row['received_s'] >= began and (not ticks or tick != ticks[-1]):
                ticks.append(tick); receipts.append(row['received_s'])
                samples.append(row['data'])
            self.owner._safe_sleep(.01)
        elapsed = [0.]
        for a,b in zip(ticks,ticks[1:]): elapsed.append(elapsed[-1]+((b-a)%(1<<24))/1000.)
        pose = dict(time_s=elapsed,
            accel_m_s2=[[s[f'acc.{a}']*G for a in 'xyz'] for s in samples],
            gyro_rad_s=np.deg2rad([[s[f'gyro.{a}'] for a in 'xyz'] for s in samples]).tolist())
        _,_,gyro = validate_pose(pose, known_reference=False)
        half_change = np.max(np.abs(gyro[:len(gyro)//2].mean(axis=0)-gyro[len(gyro)//2:].mean(axis=0)))
        if half_change > np.deg2rad(.1):
            raise RuntimeError('preflight gyro zero is still drifting; keep aircraft stationary and retry')
        self.clock_reference = {'host_receipt_monotonic_ns':[round(receipts[-1]*1e9)],
                                'cf_log_tick_ms_mod24':[ticks[-1]]}
        refresh = dict(gyro_residual_bias_rad_s=gyro.mean(axis=0).tolist(), samples=len(gyro),
            duration_s=elapsed[-1], stationary_validated=True, clock_reference=self.clock_reference,
            source_candidate_sha256=self.report['candidate_sha256'],
            accelerometer_fit_changed=False, firmware_calibration_applied=False)
        self.link.record('metadata', {**self.link.snapshot()['metadata']['data'], 'preflight_gyro_zero':refresh})
        self.report['preflight_gyro_zero'] = refresh
        self.save()

    def wait_until_fly_ready(self):
        """An arm request is not a confirmation that the supervisor permits HLC."""
        print('[estimator validation] waiting for firmware ready-to-fly', flush=True)
        began = time.monotonic()
        try:
            while True:
                snap = self.link.snapshot()
                check_snapshot(snap, self.config, flying=True, ground_z=self.owner.init_coord[2])
                info = self.check_supervisor(snap)
                if snap['health']['received_s'] >= began and info & 2 and info & 8:
                    self.link.record('event', {'name':'firmware_ready_to_fly','supervisor_info':info})
                    return
                if time.monotonic()-began > 5:
                    raise TimeoutError(f'firmware did not become ready-to-fly after arming (supervisor.info={info})')
                self.owner._safe_sleep(.02)
        except BaseException as error:
            self.owner.cf.platform.send_arming_request(False)
            self.report['error'] = str(error) or type(error).__name__
            self.save()
            raise

    def run(self):
        owner, link = self.owner, self.link
        # Controller owns takeoff and landing. This task owns only airborne sampling.
        try:
            snap = self.fresh_motion_snapshot()
            yaw = float(euler([snap['kf']['data'][f'kalman.q{i}'] for i in range(4)])[2])
            center = np.asarray(self.config['center_m'])
            for phase, start, end, duration in trajectory(center, np.asarray(owner.init_coord))[1:-1]:
                link.phase = phase
                print(f'[estimator validation] {phase}', flush=True)
                began = time.monotonic()
                if phase=='hover_start' and duration>=SETTLE_S+DURATION_S:
                    window=hover_window(began)
                    self.report['hover_roll_pitch_window']=window
                    link.record('event',window)
                    print('[estimator validation] roll/pitch: settle 2s, collect 3s for offline initialization',flush=True)
                while True:
                    snap = self.fresh_motion_snapshot()
                    u = min(1., (time.monotonic()-began)/duration)
                    position = start+(end-start)*(10*u**3-15*u**4+6*u**5)
                    measured = np.asarray(snap['vicon']['data']['position_m'])
                    distance = np.linalg.norm(measured-position)
                    if distance > .25:
                        raise RuntimeError(
                            'standard validation position deviated from target by >0.25m '
                            f'(error={distance:.3f}m; measured={measured.tolist()}; '
                            f'target={position.tolist()}; phase={phase})')
                    owner.ll_commander.send_position_setpoint(*map(float, position), yaw)
                    link.record('command', dict(position_m=position.tolist(),yaw_deg=yaw,estimator=2))
                    if u >= 1:
                        break
                    owner._safe_sleep(.02)
                if phase=='hover_start' and 'hover_roll_pitch_window' in self.report:
                    link.record('event',{'name':'hover_roll_pitch_window_frozen',
                        'end_received_s':self.report['hover_roll_pitch_window']['end_received_s'],
                        'firmware_applied':False})
                    print('[estimator validation] roll/pitch samples frozen; remaining flight is validation',flush=True)
            self.motion_completed = True
        except BaseException as error:
            self.report['error'] = str(error) or type(error).__name__
            raise
        finally:
            link.phase = 'land'
            # Controller.land replaces the trajectory and waits for its ACK
            # before releasing LL priority. Releasing here, especially on an
            # early abort, could activate a pending takeoff/go_to trajectory.

    def fresh_motion_snapshot(self):
        """Do not advance/send a new position target while callbacks catch up.

        Freshness limits remain 100 ms (300 ms health). A short receive burst
        may drain for at most another 100 ms while firmware holds the previous
        position target. Real safety errors are never retried here.
        """
        began = time.monotonic()
        first_stale = None
        while True:
            snap = self.link.snapshot(include_metadata=False)
            try:
                check_snapshot(snap, self.config, flying=True, ground_z=self.owner.init_coord[2])
            except TelemetryStaleError as error:
                if first_stale is None:
                    first_stale = error
                if time.monotonic()-began >= .1:
                    self.link.record('event', {'name':'telemetry_wait_timeout','group':error.group,
                        'age_s':error.age_s,'wait_s':time.monotonic()-began})
                    raise
                self.owner._safe_sleep(.005)
                continue
            if first_stale is not None:
                if time.monotonic()-began >= .1:
                    self.link.record('event', {'name':'telemetry_wait_timeout','group':first_stale.group,
                        'age_s':first_stale.age_s,'wait_s':time.monotonic()-began,
                        'reason':'receive catch-up deadline exceeded'})
                    raise RuntimeError('validation telemetry receive catch-up exceeded 100ms')
                self.link.record('event', {'name':'telemetry_wait_recovered','group':first_stale.group,
                    'initial_age_s':first_stale.age_s,'wait_s':time.monotonic()-began,
                    'held_previous_position_target':True})
            return snap

    def finish(self, landing_ok):
        if self.finished:
            return
        self.finished = True
        errors = []
        for block in self.blocks:
            try:
                block.stop()
                block.delete()
            except Exception as error:
                errors.append(str(error))
        with self.link.lock:
            self.journal.close()
        self.report['capture_completed'] = bool(self.motion_completed and landing_ok and not errors)
        self.report['packets_sha256'] = hashlib.sha256((self.output/'packets.jsonl').read_bytes()).hexdigest()
        if errors:
            self.report['cleanup_errors'] = errors
        self.save()
