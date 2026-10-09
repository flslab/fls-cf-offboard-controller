"""Initial position-hold sampling and frozen estimator-3 XY loading.

The normal controller still owns tracking, takeoff and landing. The worker
never sends setpoints or changes estimators; detection stays closed until ACK.
"""
from collections import deque
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import threading
import time

from Interaction.calibration_contact_logging import capture_group_manifest
from Interaction.estimator_hover_initialization import hover_window, fit_hover_roll_pitch
from Interaction.estimator_imu_calibration import write_json
from Interaction.estimator_xy_loading import (
    API, FIELDS, prepare_xy_loading, firmware_xy_coefficients, load_xy)

IMU_GROUP = 'ESTIMATOR_XY_IMU'
KF_GROUP = 'ESTIMATOR_XY_KF'
CACHE_SCHEMA = 'estimator_hover_xy_cache_v1'
MODES = ('auto', 'refresh', 'reuse')


def requested(mission):
    config = (mission or {}).get('Interaction', {}).get('config', {})
    options = config.get('level_coast') or {}
    enabled = options.get('estimator_hover_xy', False)
    if type(enabled) is not bool:
        raise ValueError('level_coast.estimator_hover_xy must be boolean')
    if enabled and options.get('estimator_hover_xy_mode', 'auto') not in MODES:
        raise ValueError('level_coast.estimator_hover_xy_mode must be auto, refresh or reuse')
    if enabled and (config.get('behavior') != 'level_coast'
                    or options.get('coast_command_mode') != 'scurve'):
        raise ValueError('estimator_hover_xy requires level_coast with scurve coasting')
    return enabled


def add_capture_logs(selected, mission, cf, args):
    if not requested(mission):
        return selected
    if getattr(args, 'calibrate', False) or not getattr(args, 'vicon', False):
        raise ValueError('estimator_hover_xy requires a normal Vicon interaction flight')
    options = mission['Interaction']['config']['level_coast']
    candidate, fingerprint, source = load_static_candidate(
        args.drone_id, options.get('estimator_imu_calibration_file'))
    saved, _ = select_saved_fit(options, candidate, fingerprint, source, args.drone_id)
    if saved is not None:
        # Cached loading does not sample or require the extra raw-IMU blocks.
        return selected
    selected = deepcopy(selected)
    selected[IMU_GROUP] = {'log_period_ms':10, **{
        f'{sensor}.{axis}':{'type':'float'} for sensor in ('acc', 'gyro') for axis in 'xyz'}}
    selected[KF_GROUP] = {'log_period_ms':20, **{f'kalman.q{i}':{'type':'float'} for i in range(4)}}
    capture_group_manifest(cf, selected, args.cf_log_period, label='estimator hover XY')
    return selected


def load_static_candidate(drone_id, configured=None):
    if configured:
        paths = [Path(configured)]
    else:
        root = Path(__file__).with_name('estimator_calibrations')/str(drone_id)
        paths = sorted(root.glob('*/fit/candidate.json'), key=lambda p:p.stat().st_mtime, reverse=True)
    for path in paths:
        raw = path.read_bytes()
        candidate = json.loads(raw)
        if candidate.get('fit_passed') is not True:
            continue
        dataset = path.parent.parent/'dataset.json'
        if (candidate.get('drone_id') != drone_id or not dataset.is_file()
                or hashlib.sha256(dataset.read_bytes()).hexdigest() != candidate.get('dataset_sha256')):
            raise ValueError('IMU calibration identity or dataset fingerprint mismatch: '+str(path))
        # Check the representable static model before any takeoff. Hover only
        # adds an offset; it cannot make a cross-axis model representable.
        import numpy as np
        A = np.asarray(candidate.get('measured_from_reference'), float)
        b = np.asarray(candidate.get('accel_bias_m_s2'), float)
        if (A.shape != (3,3) or b.shape != (3,) or not np.isfinite(A).all()
                or not np.isfinite(b).all() or not np.allclose(A,np.diag(np.diag(A)),rtol=0,atol=1e-9)
                or np.any(np.diag(A) <= 0) or np.any(np.abs(1/np.diag(A)[:2]-1) > .2)):
            raise ValueError('onboard XY initialization needs a valid diagonal static model')
        return candidate, hashlib.sha256(raw).hexdigest(), path.resolve()
    raise ValueError('no accepted static IMU calibration for '+str(drone_id))


def read_saved_fit(path, candidate, fingerprint, drone_id):
    saved = json.loads(Path(path).read_text())
    if (not isinstance(saved, dict) or saved.get('schema') != CACHE_SCHEMA
            or saved.get('drone_id') != drone_id or type(saved.get('firmware_api')) is not int
            or saved.get('firmware_api') != API or saved.get('gyro_saved') is not False
            or saved.get('independent_ground_truth') is not False
            or not isinstance(saved.get('fit'), dict)
            or not isinstance(saved.get('saved_at_utc'), str) or not saved['saved_at_utc']
            or saved.get('source_candidate_sha256') != fingerprint):
        raise ValueError('saved hover XY identity, static calibration or firmware API changed')
    expected = firmware_xy_coefficients(candidate, saved.get('fit', {}), fingerprint)
    coefficients = saved.get('coefficients')
    if (not isinstance(coefficients, dict) or set(coefficients) != set(FIELDS)
            or any(type(v) not in (int, float) for v in coefficients.values())
            or coefficients != expected):
        raise ValueError('saved hover XY coefficients do not match the accepted fit')
    return saved


def select_saved_fit(options, candidate, fingerprint, source, drone_id):
    mode = options.get('estimator_hover_xy_mode', 'auto')
    if mode not in MODES:
        raise ValueError('level_coast.estimator_hover_xy_mode must be auto, refresh or reuse')
    if mode == 'refresh':
        return None, 'manual_refresh'
    path = source.with_name('hover_xy.json')
    try:
        return read_saved_fit(path, candidate, fingerprint, drone_id), 'saved'
    except (OSError, ValueError, KeyError, TypeError) as error:
        if mode == 'reuse':
            raise ValueError('no valid saved hover XY compensation; use refresh to collect it: '+str(error)) from error
        return None, 'saved_fit_unavailable: '+str(error)


def save_fit(path, candidate, fingerprint, drone_id, fit, loaded):
    # Persist only a successful fit AND freshly acknowledged load. A failed
    # refresh leaves the previous saved set intact. write_json replaces atomically.
    coefficients = firmware_xy_coefficients(candidate, fit, fingerprint)
    if (loaded.get('firmware_applied') is not True or loaded.get('frozen') is not True
            or loaded.get('api') != API or loaded.get('coefficients') != coefficients):
        raise ValueError('cannot save hover XY before a matching firmware acknowledgement')
    saved = dict(schema=CACHE_SCHEMA, drone_id=drone_id, firmware_api=API,
        source_candidate_sha256=fingerprint, fit=deepcopy(fit), coefficients=coefficients,
        saved_at_utc=datetime.now(timezone.utc).isoformat(),
        original_load_generation=loaded['generation'], gyro_saved=False,
        independent_ground_truth=False)
    write_json(path, saved)
    return saved


class LiveHoverXY:
    def __init__(self, owner):
        self.owner = owner
        options = owner.mission['Interaction']['config']['level_coast']
        self.candidate, self.fingerprint, self.source = load_static_candidate(
            owner.args.drone_id, options.get('estimator_imu_calibration_file'))
        self.cache_path = self.source.with_name('hover_xy.json')
        self.saved, self.selection = select_saved_fit(
            options, self.candidate, self.fingerprint, self.source, owner.args.drone_id)
        if self.saved is None:
            groups = getattr(owner.log_manager, 'cf_log_data', None)
            if isinstance(groups, dict) and not {IMU_GROUP, KF_GROUP}.issubset(groups):
                raise RuntimeError('saved hover XY changed after log setup; restart to prepare sampling')
        # Fail before arming when the paired firmware is missing or not reset.
        prepare_xy_loading(owner.cf)
        self.lock = threading.Lock()
        self.rows = deque(maxlen=6000)
        self.window = None
        self.worker = None
        self.result = None
        self.failure = None
        self.closed = False
        self.ready = False
        self.output = Path(owner.args.log_dir)/(owner.args.tag+'_estimator_xy')
        self.output.mkdir(parents=True, exist_ok=False)
        write_json(self.output/'source.json', dict(candidate_path=str(self.source),
            candidate_sha256=self.fingerprint, drone_id=owner.args.drone_id,
            cache_path=str(self.cache_path), selection=self.selection))
        self.unsubscribes = ([owner.log_manager.add_cf_packet_listener(self.on_cf),
                             owner.log_manager.add_mocap_frame_listener(self.on_mocap)]
                            if self.saved is None else [])
        owner.cf._estimator_hover_xy = self

    def record(self, group, data, received, tick=None):
        with self.lock:
            if self.closed or self.worker is not None or self.saved is not None:
                return
            self.rows.append(dict(group=group, data=data, received_s=received,
                cf_log_tick_ms_mod24=tick, phase='hover_start' if self.window else 'preflight'))

    def on_cf(self, packet):
        group = {IMU_GROUP:'imu', KF_GROUP:'kf'}.get(packet.group)
        if group:
            self.record(group, dict(packet.data), packet.host_receive_monotonic_s,
                        packet.cf_timestamp_ms)

    def on_mocap(self, packet):
        if packet.group == 'frames':
            self.record('vicon', dict(position_m=list(packet.data['tvec'])),
                        packet.host_receive_monotonic_s)

    def update(self, target, *, can_begin, now):
        if self.failure is not None:
            raise RuntimeError('estimator hover XY initialization failed: '+self.failure)
        if self.result is not None:
            # Let corrected propagation replace old shadow history before use.
            self.ready = now-self.result['applied_monotonic_s'] >= .3
            return self.ready
        if self.saved is not None:
            if can_begin and self.worker is None:
                self.worker = threading.Thread(target=self.load_saved, daemon=True)
                self.worker.start()
            return False
        if self.window is None and can_begin:
            self.window = hover_window(now)
            self.record('event', dict(self.window), now)
            print('[interaction] estimator XY: settle 2s, sample 3s; keep hands off', flush=True)
        self.record('command', dict(position_m=list(map(float, target)), estimator=2), now)
        if self.window and now >= self.window['end_received_s'] and self.worker is None:
            with self.lock:
                frozen = list(self.rows)
                self.worker = threading.Thread(target=self.fit_and_load, args=(frozen,), daemon=True)
            self.worker.start()
        return False

    def commit(self, action):
        with self.lock:
            if self.closed:
                raise RuntimeError('hover XY loading cancelled')
            action()

    def fit_and_load(self, rows):
        try:
            (self.output/'packets.jsonl').write_text(''.join(json.dumps(r, allow_nan=False)+'\n' for r in rows))
            fit = fit_hover_roll_pitch(rows, self.candidate, self.fingerprint)
            write_json(self.output/'fit.json', fit)
            if not fit['accepted']:
                raise ValueError('; '.join(fit['failures']))
            coefficients = firmware_xy_coefficients(self.candidate, fit, self.fingerprint)
            result = load_xy(self.owner.cf, coefficients, commit_guard=self.commit)
            save_fit(self.cache_path, self.candidate, self.fingerprint,
                     self.owner.args.drone_id, fit, result)
            result.update(reused_from_cache=False, cache_path=str(self.cache_path))
            write_json(self.output/'loaded.json', result)
            with self.lock:
                if not self.closed:
                    self.result = result
            print('[interaction] estimator XY loaded, saved and frozen', flush=True)
        except Exception as error:
            self.failure = str(error)
            write_json(self.output/'failure.json', dict(error=str(error), firmware_applied=None))

    def load_saved(self):
        try:
            print('[interaction] estimator XY: loading saved compensation', flush=True)
            write_json(self.output/'fit.json', self.saved['fit'])
            result = load_xy(self.owner.cf, self.saved['coefficients'], commit_guard=self.commit)
            result.update(reused_from_cache=True, cache_path=str(self.cache_path),
                          saved_at_utc=self.saved['saved_at_utc'])
            write_json(self.output/'loaded.json', result)
            with self.lock:
                if not self.closed:
                    self.result = result
            print('[interaction] estimator XY saved compensation loaded and frozen', flush=True)
        except Exception as error:
            self.failure = str(error)
            write_json(self.output/'failure.json', dict(error=str(error), firmware_applied=None))

    def close(self):
        with self.lock:
            self.closed = True
        for unsubscribe in self.unsubscribes:
            unsubscribe()
