"""Offline real-C estimator-3 replay, raw versus calibrated IMU inputs.

Default-estimator attitude is a relative evaluation reference only. It seeds
each replay segment once and is never used as an ongoing attitude observation.
"""
import argparse
from collections import deque
import ctypes as C
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np

from Interaction.estimator_imu_calibration import G, write_json
from Interaction.estimator_validation_flight import euler

NATIVE = Path(__file__).with_name('native_estimator3')


def build(output):
    manifest = json.loads((NATIVE/'manifest.json').read_text())
    for name, row in manifest['files'].items():
        if hashlib.sha256((NATIVE/name).read_bytes()).hexdigest() != row['sha256']:
            raise ValueError('native estimator source fingerprint changed: '+name)
    compiler = shutil.which(os.environ.get('CC', 'cc'))
    if compiler is None:
        raise RuntimeError('offline replay needs a C compiler on the analysis computer (cc/clang/gcc)')
    library = Path(output)/('estimator3.dylib' if sys.platform == 'darwin' else 'estimator3.so')
    command = [compiler, '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror', '-shared', '-fPIC',
               '-I'+str(NATIVE), str(NATIVE/'bridge.c'), str(NATIVE/'post_release_inertial_ekf.c'),
               str(NATIVE/'post_release_vicon_velocity_kf.c'), '-lm', '-o', str(library)]
    result = subprocess.run(command, capture_output=True, text=True, timeout=60)
    if result.returncode:
        raise RuntimeError('native estimator build failed: '+result.stderr[-4000:])
    write_json(Path(output)/'build.json', {**manifest, 'command': command,
        'bridge_sha256': hashlib.sha256((NATIVE/'bridge.c').read_bytes()).hexdigest()})
    return library


class Native:
    def __init__(self, library):
        self.lib = C.CDLL(str(Path(library).resolve()))
        ptr = C.POINTER(C.c_float)
        self.lib.native_init.argtypes = [ptr, ptr, ptr, ptr, ptr, C.c_uint]
        self.lib.native_propagate.argtypes = [ptr, ptr, C.c_uint]
        self.lib.native_frontend.argtypes = [ptr, C.c_uint, C.c_float]
        self.lib.native_state.argtypes = [ptr]
        self.lib.native_previous_gyro.argtypes = [ptr]

    @staticmethod
    def vec(values):
        return (C.c_float*len(values))(*values)

    def seed(self, p, v, q, t, gyro):
        if not self.lib.native_init(self.vec(p), self.vec(v), self.vec(q), self.vec([0, 0, 0]), self.vec([0, 0, 0]), t):
            raise RuntimeError('native estimator initialization failed')
        self.lib.native_previous_gyro(self.vec(gyro))

    def step(self, gyro, acc, t):
        return self.lib.native_propagate(self.vec(gyro), self.vec(acc), t)

    def position(self, p, t):
        return self.lib.native_frontend(self.vec(p), t, C.c_float(.999))

    def state(self):
        out = (C.c_float*24)()
        self.lib.native_state(out)
        return np.array(out, float)


def replay(rows, candidate, native, *, corrected):
    latest, pending, output, rejected = {}, [], [], []
    histories = {name: deque(maxlen=30) for name in ('kf', 'state')}
    anchor, previous_tick, elapsed, previous_t = None, None, 0, None
    first_tick = None
    previous_imu = None
    ready, active, segment = False, False, 0
    gyro_fit = candidate.get('gyro_calibration')
    ram_header = next((row['data']['ram_trace'] for row in rows
                       if row['group'] == 'metadata' and 'ram_trace' in row['data']), None)
    gap_limit_us = max(2000, 3_000_000 // ram_header['imu_hz']) if ram_header else 20000
    A = np.asarray(candidate['measured_from_reference'], float)
    b = np.asarray(candidate['accel_bias_m_s2'], float)
    bg = np.asarray(candidate['gyro_residual_bias_rad_s'], float)
    for row in rows:
        group = row['group']
        latest[group] = row
        if group in histories:
            histories[group].append(row)
        if group == 'event' and row['data'].get('name') == 'capture_ready':
            ready = True
        if group == 'vicon' and ready:
            pending.append(row)
        if group != 'imu' or not ready:
            continue
        tick = row.get('device_time_us') if ram_header else row['cf_log_tick_ms_mod24']
        modulus = 1 << 32 if ram_header else 1 << 24
        if type(tick) is not int or (not ram_header and not 0 <= tick < modulus):
            raise ValueError('invalid device IMU log tick')
        if tick == previous_tick and previous_imu == row['data']:
            # A retried CRTP packet / overdue SITL log worker can deliver the
            # exact same device sample twice. Do not integrate it twice or
            # invent elapsed time. Conflicting same-tick samples still fail.
            rejected.append({'time_us':1_000_000+elapsed*(1 if ram_header else 1000),
                             'reason':'identical duplicate IMU packet skipped'})
            continue
        if previous_tick is not None:
            delta = (tick-previous_tick) % modulus
            if not 0 < delta < modulus//2:
                raise ValueError('duplicate/reordered device tick or firmware reset')
            elapsed += delta
        previous_tick = tick
        previous_imu = row['data']
        if anchor is None:
            anchor = row['received_s']
            first_tick = tick
        t = 1_000_000+elapsed*(1 if ram_header else 1000)
        references = {}
        for name, history in histories.items():
            for sample in reversed(history):
                sample_tick = sample.get('device_time_us') if ram_header else sample.get('cf_log_tick_ms_mod24')
                if type(sample_tick) is not int:
                    continue
                device_age = (tick-sample_tick) % modulus
                if 0 <= device_age <= (60000 if ram_header else 60) and 0 <= row['received_s']-sample['received_s'] <= .06:
                    references[name] = sample
                    break
        if len(references) != 2:
            active = False
            rejected.append({'time_us': t, 'reason': 'no causal fresh default-estimator seed/reference'})
            continue
        reference = [references['kf']['data'][f'kalman.q{i}'] for i in range(4)]
        state = references['state']['data']
        acc = np.array([row['data'][f'acc.{a}'] for a in 'xyz'])*G
        gyro = np.deg2rad([row['data'][f'gyro.{a}'] for a in 'xyz'])
        if not np.isfinite(acc).all() or not np.isfinite(gyro).all():
            raise ValueError('nonfinite IMU measurement')
        if corrected:
            acc = np.linalg.solve(A, acc-b)
            gyro = np.linalg.solve(gyro_fit['measured_from_reference'], gyro-bg) if gyro_fit else gyro-bg
        if not active or previous_t is None or t-previous_t > gap_limit_us:
            native.seed([state[f'stateEstimate.{a}'] for a in 'xyz'],
                        [state[f'stateEstimate.v{a}'] for a in 'xyz'], reference, t, gyro)
            segment += 1
            active = True
            pending = []
        elif not native.step(gyro, acc, t):
            active = False
            rejected.append({'time_us': t, 'reason': 'native filter invalid; next segment will reseed'})
            previous_t = t
            continue
        previous_t = t
        while pending:
            observation = pending[0]
            stamp = (1_000_000+observation['device_time_us']-first_tick if ram_header else
                     round(1_000_000+(observation['received_s']-anchor)*1e6))
            if stamp > t:
                break
            pending.pop(0)
            if stamp < 1_000_000 or t-stamp > 40000:
                rejected.append({'time_us': t, 'reason': 'position receipt epoch outside causal 40ms window'})
                continue
            native.position(observation['data']['position_m'], stamp)
        estimate = native.state()
        rpy, ref = euler(estimate[6:10]), euler(reference)
        error = (rpy-ref+180)%360-180
        output.append({'time_s': elapsed/(1e6 if ram_header else 1000.), 'phase': row['phase'], 'segment': segment,
            'reference_rpy_deg': ref.tolist(), 'rpy_deg': rpy.tolist(), 'error_deg': error.tolist(),
            'position_m': estimate[:3].tolist(), 'velocity_m_s': estimate[3:6].tolist(),
            'gyro_bias_residual_rad_s': estimate[10:13].tolist(), 'accel_bias_residual_m_s2': estimate[13:16].tolist(),
            'position_fusions': int(estimate[20]), 'position_rejections': int(estimate[21])})
    if not output:
        raise ValueError('no replayable IMU samples after capture_ready with causal default-estimator state')
    evaluation = [r for r in output if r['phase'] not in ('preflight', 'takeoff', 'land', 'abort_land')]
    stats = {}
    for phase in sorted({r['phase'] for r in evaluation}):
        values = np.asarray([r['error_deg'] for r in evaluation if r['phase'] == phase])
        stats[phase] = {'samples': len(values), 'rmse_rpy_deg': np.sqrt(np.mean(values**2, axis=0)).tolist(),
                       'mean_error_rpy_deg': np.mean(values, axis=0).tolist()}
    return {'samples': output, 'by_phase': stats, 'segments': segment, 'rejected': rejected,
            'gap_limit_us': gap_limit_us, 'ram_capture': ram_header}


def candidate_for_capture(rows, candidate, candidate_sha256):
    """A measured preflight zero replaces only the boot-dependent gyro offset."""
    from copy import deepcopy
    refreshes = [r['data']['preflight_gyro_zero'] for r in rows
                 if r['group']=='metadata' and 'preflight_gyro_zero' in r['data']]
    effective = deepcopy(candidate)
    if not refreshes:
        return effective, None
    refresh = refreshes[-1]
    bias = np.asarray(refresh.get('gyro_residual_bias_rad_s'), dtype=float)
    if (refresh.get('source_candidate_sha256') != candidate_sha256
            or refresh.get('stationary_validated') is not True
            or refresh.get('samples',0) < 200 or refresh.get('duration_s',0) < 2
            or refresh.get('accelerometer_fit_changed') is not False
            or refresh.get('firmware_calibration_applied') is not False
            or bias.shape != (3,) or not np.isfinite(bias).all()
            or np.max(np.abs(bias)) > np.deg2rad(2)):
        raise ValueError('preflight gyro-zero metadata is invalid or belongs to another calibration')
    effective['gyro_residual_bias_rad_s'] = bias.tolist()
    return effective, refresh


def evaluation_summary(result):
    samples=[r for r in result['samples'] if r['phase'] in result['by_phase']]
    if not samples:
        raise ValueError('no held-out flight samples for hover initialization')
    errors=np.asarray([r['error_deg'] for r in samples])
    return {'samples':len(samples),'segments':result['segments'],
            'rmse_rpy_deg':np.sqrt(np.mean(errors**2,axis=0)).tolist(),
            'mean_error_rpy_deg':errors.mean(axis=0).tolist()}


def compare_hover_initialization(rows, candidate, candidate_sha256, native, output):
    from Interaction.estimator_hover_initialization import (
        fit_hover_roll_pitch, apply_hover_roll_pitch, held_out_hover_rows)
    fit=fit_hover_roll_pitch(rows,candidate,candidate_sha256)
    directory=output/'hover_initialization'
    directory.mkdir()
    write_json(directory/'relative_fit.json',fit)
    comparison={'fit':fit,'completed':False,'firmware_applied':False,
                'independent_ground_truth':False,'yaw_calibrated':False}
    if not fit['accepted']:
        write_json(directory/'comparison.json',comparison)
        return comparison
    initialized=apply_hover_roll_pitch(candidate,fit,candidate_sha256)
    write_json(directory/'effective_candidate.json',{
        **initialized,'relative_hover_initialization':fit,
        'firmware_applied':False,'deployment_approved':False})
    held_out=held_out_hover_rows(rows,fit)
    try:
        baseline=replay(held_out,candidate,native,corrected=True)
        corrected=replay(held_out,initialized,native,corrected=True)
        baseline_summary=evaluation_summary(baseline)
        corrected_summary=evaluation_summary(corrected)
        identity=lambda r:[(s['time_s'],s['phase'],s['segment']) for s in r['samples']]
        if (identity(baseline)!=identity(corrected) or baseline['segments']!=1):
            raise ValueError('hover comparison needs matching samples and one uninterrupted segment per arm')
        comparison.update(completed=True,training_excluded=True,
                          evaluation_start_received_s=fit['window']['end_received_s'],
                          static_summary=baseline_summary,initialized_summary=corrected_summary,
                          static=baseline,initialized=corrected)
        comparison['roll_pitch_improved_relative_to_default'] = bool(np.all(
            np.asarray(corrected_summary['rmse_rpy_deg'])[:2] < np.asarray(baseline_summary['rmse_rpy_deg'])[:2]))
    except ValueError as error:
        comparison['error']=str(error)
    # Fit acceptance and held-out comparison are separate; neither activates it.
    write_json(directory/'comparison.json',{k:v for k,v in comparison.items() if k not in ('static','initialized')})
    return comparison


def compare_onboard_xy(rows, candidate, fingerprint, native, output):
    """Replay the actual loading interface: raw Z/gyro, calibrated XY only."""
    from Interaction.estimator_hover_initialization import fit_hover_roll_pitch, held_out_hover_rows
    from Interaction.estimator_xy_loading import firmware_xy_coefficients, onboard_xy_candidate
    fit = fit_hover_roll_pitch(rows, candidate, fingerprint)
    result = dict(completed=False, firmware_applied=False,
                  independent_ground_truth=False, z_changed=False, gyro_changed=False,
                  yaw_calibrated=False, training_excluded=True, fit=fit)
    try:
        coefficients = firmware_xy_coefficients(candidate, fit, fingerprint)
        selected = held_out_hover_rows(rows, fit)
        raw = replay(selected, candidate, native, corrected=False)
        loaded = replay(selected, onboard_xy_candidate(coefficients), native, corrected=True)
        identity = lambda arm: [(s['time_s'],s['phase'],s['segment']) for s in arm['samples']]
        if raw['segments'] != 1 or identity(raw) != identity(loaded):
            raise ValueError('onboard XY comparison requires one matching uninterrupted segment')
        result.update(completed=True, coefficients=coefficients,
                      raw_summary=evaluation_summary(raw), initialized_summary=evaluation_summary(loaded))
    except (ValueError, KeyError) as error:
        result['error'] = str(error)
    write_json(Path(output)/'onboard_xy_comparison.json', result)
    return result


def run_replay(packets, candidate_path, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    candidate = json.loads(Path(candidate_path).read_text())
    if candidate.get('fit_passed') is not True:
        raise ValueError('replay requires a successful fixture fit')
    if candidate.get('gyro_calibration') and candidate['gyro_calibration'].get('accepted') is not True:
        raise ValueError('gyro fit has not passed its independent rotation checks')
    raw = Path(packets).read_bytes()
    rows = [json.loads(line) for line in raw.splitlines()]
    candidate, refresh = candidate_for_capture(rows, candidate,
        hashlib.sha256(Path(candidate_path).read_bytes()).hexdigest())
    receipts = [r['received_s'] for r in rows]
    if any(b < a for a, b in zip(receipts, receipts[1:])):
        # Callback clocks are taken before acquiring the shared writer lock.
        # Stable receipt sorting restores causal ordering; never sort by phase.
        rows.sort(key=lambda r: r['received_s'])
    native = Native(build(output))
    baseline = replay(rows, candidate, native, corrected=False)
    corrected = replay(rows, candidate, native, corrected=True)
    candidate_sha256=hashlib.sha256(Path(candidate_path).read_bytes()).hexdigest()
    hover=compare_hover_initialization(rows,candidate,candidate_sha256,native,output)
    onboard_xy=compare_onboard_xy(rows,candidate,candidate_sha256,native,output)
    report = {'schema': 'offline_estimator3_validation_v1', 'completed': True,
        'reference': 'default_estimator', 'independent_ground_truth': False,
        'onboard_kernel_identity_verified': False,
        'onboard_estimator3_enabled': False, 'firmware_calibration_applied': False,
        'calibration_applied_offline': True, 'gyro_matrix_applied': bool(candidate.get('gyro_calibration')),
        'calibration_method': candidate.get('calibration_method', 'six_face'),
        'accel_alignment_calibrated': candidate.get('accel_alignment_calibrated', True),
        'preflight_gyro_zero': refresh,
        'packets_sha256': hashlib.sha256(raw).hexdigest(),
        'candidate_sha256': candidate_sha256,
        'raw': baseline, 'corrected': corrected,
        'hover_roll_pitch':hover,
        'onboard_xy':onboard_xy,
        'limitations': ['Actual frozen firmware C kernel at logged IMU rate; not bit-exact high-rate onboard replay.',
            'Vicon positions use host receipt timing mapped to device log ticks; capture delay is not identified.',
            'Default estimator shares sensors and supplies segment initialization; agreement is not physical accuracy.',
            'Segments restart after >20ms IMU gaps; compare segment counts and do not conceal resets.',
            'No active estimator-3 controller or contact interaction is validated by this replay.']}
    if baseline['ram_capture']:
        report['ram_capture'] = baseline['ram_capture']
        report['limitations'][1] = 'RAM trace uses firmware receipt/acquisition timestamps; camera capture delay is not identified.'
        report['limitations'][3] = f'Segments restart after >{baseline["gap_limit_us"]}us IMU gaps or unavailable references; inspect segment counts.'
    if report['calibration_method'] == 'gravity_norm':
        report['limitations'].append('Gravity-norm correction fits diagonal axis scale and bias only; sensor/body rotation and cross-axis terms remain uncalibrated.')
    write_json(output/'report.json', report)
    lines = ['# Offline estimator 3 comparison', '', 'Reference: default estimator (relative reference, not independent truth).',
             '', f'Gyro matrix applied offline: {report["gyro_matrix_applied"]}', '',
             '| Phase | Samples raw / corrected | Raw roll/pitch/yaw RMSE (deg) | Corrected roll/pitch/yaw RMSE (deg) |', '| --- | --- | --- | --- |']
    for phase in sorted(baseline['by_phase'].keys() & corrected['by_phase'].keys()):
        values = [' / '.join(f'{v:.3f}' for v in result['by_phase'][phase]['rmse_rpy_deg']) for result in (baseline, corrected)]
        counts = f'{baseline["by_phase"][phase]["samples"]} / {corrected["by_phase"][phase]["samples"]}'
        lines.append(f'| {phase} | {counts} | {values[0]} | {values[1]} |')
    lines += ['', f'Segments raw/corrected: {baseline["segments"]}/{corrected["segments"]}', '', *report['limitations']]
    lines += ['', '## Relative hover roll/pitch initialization', '',
              'First hover: settle 2 seconds, train 3 seconds; subsequent data only for comparison.',
              'Only X/Y specific-force offset changes. Z correction and gyro parameters are unchanged.',
              'Yaw is not calibrated; its output may change indirectly through coupled ESKF updates.',
              'Default estimator 2 is a shared-sensor relative reference, not independent attitude truth.',
              'Source static fit and firmware are unchanged. No deployment approval is implied.', '',
              f'Hover fit accepted: {hover["fit"]["accepted"]}; held-out comparison complete: {hover["completed"]}.']
    if hover['completed']:
        lines += ['', '| Held-out comparison | Samples | Roll RMSE (deg) | Pitch RMSE (deg) | Yaw RMSE (deg) |',
                  '| --- | --- | --- | --- | --- |']
        for name,key in [('Static correction','static_summary'),('Static + hover X/Y initialization','initialized_summary')]:
            stats=hover[key]
            values=' | '.join(f'{v:.3f}' for v in stats['rmse_rpy_deg'])
            lines.append(f'| {name} | {stats["samples"]} | {values} |')
    else:
        lines += ['', *hover['fit']['failures']]
        if hover.get('error'):
            lines.append(hover['error'])
    (output/'report.md').write_text('\n'.join(lines)+'\n')
    with (output/'attitude.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['mode', 'time_s', 'phase', 'segment', 'roll_deg', 'pitch_deg', 'yaw_deg',
                         'reference_roll_deg', 'reference_pitch_deg', 'reference_yaw_deg'])
        arms=[('raw',baseline),('corrected',corrected)]
        if hover['completed']:
            arms += [('static_holdout',hover['static']),('hover_xy_holdout',hover['initialized'])]
        for name, result in arms:
            for row in result['samples']:
                writer.writerow([name, row['time_s'], row['phase'], row['segment'], *row['rpy_deg'], *row['reference_rpy_deg']])
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('packets', type=Path)
    parser.add_argument('--candidate', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        result = run_replay(args.packets, args.candidate, args.output)
        print(f'Offline estimator-3 replay completed: {args.output}; gyro matrix applied: {result["gyro_matrix_applied"]}')
        hover=result['hover_roll_pitch']
        if hover['completed']:
            before=hover['static_summary']['rmse_rpy_deg'];after=hover['initialized_summary']['rmse_rpy_deg']
            print(f'Hover roll/pitch initialization (offline only): roll {before[0]:.3f} -> {after[0]:.3f} deg, '
                  f'pitch {before[1]:.3f} -> {after[1]:.3f} deg; yaw not calibrated.')
        else:
            print('Hover roll/pitch initialization unavailable: '+('; '.join(hover['fit']['failures']) or hover.get('error','no held-out comparison')))
        return 0
    except (ValueError, RuntimeError, OSError, KeyError, subprocess.SubprocessError) as error:
        print(f'Offline replay failed: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
