"""Known-fixture calibration of the processed IMU consumed by the estimators.

No estimator output is used as attitude truth. The map is measured=A*reference+b;
correction is solve(A, measured-b). This module never commands the aircraft.
"""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import tempfile

import numpy as np

G = 9.81
SCHEMA = 'estimator_processed_imu_fixture_v1'
LIMITS = {
    'minimum_samples_per_pose': 200,
    'minimum_pose_duration_s': 2.,
    'maximum_sample_gap_s': .05,
    'maximum_accel_std_m_s2': .15,
    'maximum_accel_half_window_change_m_s2': .12,
    'maximum_gyro_std_deg_s': .8,
    'maximum_residual_gyro_bias_deg_s': 2.,
    'maximum_gyro_pose_change_deg_s': .3,
    'maximum_fit_rmse_m_s2': .12,
    'maximum_validation_rmse_m_s2': .12,
    'maximum_validation_pose_error_m_s2': .15,
    'maximum_validation_angle_deg': 1.,
    'maximum_validation_gyro_bias_deg_s': .3,
}


def default_plan():
    """Body +axis pointing vertically UP gives positive specific force on it."""
    return [
        {'id': f'{role}_{axis}{sign_name}', 'role': role,
         'instruction': f'Body {sign_name}{axis.upper()} points vertically UP',
         'reference_force_m_s2': [sign * G if i == j else 0. for i in range(3)]}
        for role in ('train', 'validation')
        for j, axis in enumerate('xyz')
        for sign, sign_name in ((1, '+'), (-1, '-'))
    ]


def _array(value, shape, name):
    result = np.asarray(value, dtype=float)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f'{name}: expected finite shape {shape}')
    return result


def validate_pose(pose, *, known_reference=True):
    """Check the whole contiguous window; never select its quietest subset."""
    t = np.asarray(pose['time_s'], dtype=float)
    n = len(t)
    if t.ndim != 1 or n < LIMITS['minimum_samples_per_pose'] or not np.isfinite(t).all():
        raise ValueError('pose needs at least 200 finite timestamped samples')
    dt = np.diff(t)
    if np.any(dt <= 0) or np.max(dt) > LIMITS['maximum_sample_gap_s'] + 1e-9:
        raise ValueError('pose has duplicate/reversed timestamps or a sample gap >50 ms')
    if t[-1] - t[0] < LIMITS['minimum_pose_duration_s']:
        raise ValueError('pose duration must be at least 2 seconds')
    acc = _array(pose['accel_m_s2'], (n, 3), 'accel')
    gyro = _array(pose['gyro_rad_s'], (n, 3), 'gyro')
    ref = None
    if known_reference:
        ref = _array(pose['reference_force_m_s2'], (3,), 'reference force')
        if abs(np.linalg.norm(ref) - G) > .05:
            raise ValueError('reference must be gravity in an independently known static pose')
    if np.any(np.abs(acc) > 2 * G) or np.any(np.abs(gyro) > np.deg2rad(5)):
        raise ValueError('pose is moving or accelerometer is outside the static range')
    if np.max(np.std(acc, axis=0)) > LIMITS['maximum_accel_std_m_s2']:
        raise ValueError('accelerometer motion/noise exceeds the static limit')
    half_change = np.mean(acc[:n//2], axis=0) - np.mean(acc[n//2:], axis=0)
    if np.max(np.abs(half_change)) > LIMITS['maximum_accel_half_window_change_m_s2']:
        raise ValueError('accelerometer drifts across the pose window')
    if np.max(np.std(gyro, axis=0)) > np.deg2rad(LIMITS['maximum_gyro_std_deg_s']):
        raise ValueError('gyro motion/noise exceeds the static limit')
    if np.max(np.abs(np.mean(gyro, axis=0))) > np.deg2rad(2):
        raise ValueError('residual gyro bias exceeds 2 deg/s')
    return ref, acc, gyro


def _coverage(refs, role):
    design = np.c_[np.asarray(refs) / G, np.ones(len(refs))]
    condition = float(np.linalg.cond(design))
    if len(refs) < 6 or np.linalg.matrix_rank(design) < 4 or condition > 5:
        raise ValueError(f'{role}: need six diverse known poses; one hover is unobservable')
    # All three axes must have both positive and negative gravity excitation.
    directions = np.asarray(refs) / G
    if np.any(np.max(directions, axis=0) < .5) or np.any(np.min(directions, axis=0) > -.5):
        raise ValueError(f'{role}: missing positive/negative axis coverage')
    return condition


def _metrics(acc, gyro, ref, matrix, ba, bg):
    corrected = np.linalg.solve(matrix, (acc - ba).T).T
    mean = np.mean(corrected, axis=0)

    def angle(vector):
        norm = np.linalg.norm(vector)
        if norm < 1e-9:
            return 180.  # zero force has no defined gravity direction; fail validation
        cosine = np.dot(vector, ref) / (norm * np.linalg.norm(ref))
        return float(np.rad2deg(np.arccos(np.clip(cosine, -1, 1))))

    return {
        'corrected_rmse_m_s2': float(np.sqrt(np.mean((corrected - ref) ** 2))),
        'corrected_mean_error_m_s2': float(np.linalg.norm(mean - ref)),
        'corrected_angle_deg': angle(mean),
        'uncorrected_rmse_m_s2': float(np.sqrt(np.mean((acc - ref) ** 2))),
        'uncorrected_angle_deg': angle(np.mean(acc, axis=0)),
        'corrected_gyro_mean_deg_s': np.rad2deg(np.mean(gyro, axis=0) - bg).tolist(),
        'sample_count': len(acc),
    }


def fit_dataset(document, *, require_validation=True):
    """Fit ONLY training windows, then independently accept/reject on validation."""
    if isinstance(document, dict) and document.get('reference_source') == 'gravity_magnitude':
        from Interaction.estimator_gravity_calibration import fit_gravity_dataset
        return fit_gravity_dataset(document, require_validation=require_validation)
    if not isinstance(document, dict) or document.get('schema') != SCHEMA or document.get('status') != 'complete':
        raise ValueError('need a complete fixture dataset with the supported schema')
    for key in ('drone_id', 'firmware_id', 'fixture_id', 'reference_note'):
        if not isinstance(document.get(key), str) or not document[key].strip():
            raise ValueError(f'missing calibration provenance: {key}')
    if document.get('reference_source') != 'independent_fixture':
        raise ValueError('estimator attitude cannot be used as calibration truth')
    if document.get('sensor_frame') != 'driver_processed_body' or document.get('motors_off_confirmed') is not True:
        raise ValueError('requires driver-processed body IMU and motors-off confirmation')
    capture = document.get('capture', {})
    before = capture.get('firmware_parameters_before', {}).get('imu_sensors')
    after = capture.get('firmware_parameters_after', {}).get('imu_sensors')
    if before is not None and after is not None and before != after:
        raise ValueError('IMU sensor parameters changed during collection')
    poses = document.get('poses', [])
    ids = [pose['id'] for pose in poses]
    if len(ids) != len(set(ids)):
        raise ValueError('pose IDs must be unique')
    grouped = {'train': [], 'validation': []}
    sample_fingerprints = set()
    for pose in poses:
        if pose.get('role') not in grouped:
            raise ValueError('pose role must be train or validation')
        reference, acc, gyro = validate_pose(pose)
        fingerprint = hashlib.sha256(acc.tobytes() + gyro.tobytes()).hexdigest()
        if fingerprint in sample_fingerprints:
            raise ValueError('reused sample window; validation needs an independent collection')
        sample_fingerprints.add(fingerprint)
        grouped[pose['role']].append((pose, reference, acc, gyro))
    conditions = {role: _coverage([row[1] for row in rows], role)
                  for role, rows in grouped.items() if rows or require_validation}
    # Each pose contributes equally, even when radio delivery rates differ.
    train = grouped['train']
    ref = np.asarray([row[1] for row in train])
    means = np.asarray([np.mean(row[2], axis=0) for row in train])
    coefficients = np.linalg.lstsq(np.c_[ref / G, np.ones(len(ref))], means, rcond=None)[0]
    matrix, ba = coefficients[:3].T / G, coefficients[3]
    singular = np.linalg.svd(matrix, compute_uv=False)
    if np.linalg.det(matrix) <= 0 or np.any((singular < .8) | (singular > 1.2)):
        raise ValueError('invalid accelerometer scale/reflection; check pose labels and units')
    if np.max(np.abs(ba)) > .5:
        raise ValueError('accelerometer bias exceeds 0.5 m/s²')
    u, _, vt = np.linalg.svd(matrix)
    rotation = u @ vt
    angle = float(np.rad2deg(np.arccos(np.clip((np.trace(rotation)-1)/2, -1, 1))))
    if angle > 10:
        raise ValueError('alignment exceeds 10 degrees; check fixture/body-axis convention')
    gyro_means = np.asarray([np.mean(row[3], axis=0) for row in train])
    bg = np.mean(gyro_means, axis=0)
    if np.max(np.ptp(gyro_means, axis=0)) > np.deg2rad(.3):
        raise ValueError('gyro zero changes across training poses; warm up and repeat')
    metrics = {pose['id']: {'role': pose['role'], **_metrics(acc, gyro, reference, matrix, ba, bg)}
               for pose, reference, acc, gyro in sum(grouped.values(), [])}
    failures = []
    for name, row in metrics.items():
        rmse_limit = LIMITS['maximum_fit_rmse_m_s2'] if row['role'] == 'train' else LIMITS['maximum_validation_rmse_m_s2']
        if row['corrected_rmse_m_s2'] > rmse_limit:
            failures.append(f'{name}: corrected acceleration RMSE > {rmse_limit}')
        if row['role'] == 'validation':
            if (row['corrected_mean_error_m_s2'] > LIMITS['maximum_validation_pose_error_m_s2']
                    or row['corrected_angle_deg'] > LIMITS['maximum_validation_angle_deg']):
                failures.append(f'{name}: independent validation error exceeds limit')
            if max(abs(v) for v in row['corrected_gyro_mean_deg_s']) > LIMITS['maximum_validation_gyro_bias_deg_s']:
                failures.append(f'{name}: residual gyro zero changed after fitting')
    return {
        'schema': 'estimator_processed_imu_calibration_v1',
        'accepted': not failures and bool(grouped['validation']), 'failures': failures,
        'fit_passed': not failures,
        'independent_static_validation_passed': not failures and bool(grouped['validation']),
        'drone_id': document['drone_id'], 'firmware_id': document['firmware_id'],
        'fixture_id': document['fixture_id'], 'reference_note': document['reference_note'],
        'provenance_documented': all(document[key].strip().lower() != 'unknown'
                                     for key in ('firmware_id','fixture_id','reference_note')),
        'sensor_frame': 'driver_processed_body',
        'calibration_method': 'six_face', 'reference_source': 'independent_fixture',
        'accel_alignment_calibrated': True, 'accel_cross_axis_calibrated': True,
        'measured_from_reference': matrix.tolist(), 'accel_bias_m_s2': ba.tolist(),
        'gyro_residual_bias_rad_s': bg.tolist(),
        'correction': 'acc_corrected = solve(measured_from_reference, acc_measured - accel_bias_m_s2); gyro_corrected = gyro_measured - gyro_residual_bias_rad_s',
        'alignment_rotation_deg': angle, 'singular_values': singular.tolist(),
        'reference_conditions': conditions, 'pose_metrics': metrics, 'limits': dict(LIMITS),
        'gyro_axis_scale_calibrated': False, 'vicon_delay_calibrated': False,
        'firmware_applied': False, 'flight_validated': False,
        'note': 'Known fixture is operator-supplied truth. Repeat poses cannot detect a shared fixture error. Residual gyro zero is boot/temperature dependent. No onboard estimator loads this file.',
    }


def write_json(path, document):
    """Atomic replacement inside a new session; callers never reuse session dirs."""
    path = Path(path)
    payload = json.dumps(document, indent=2, allow_nan=False) + '\n'
    fd, temporary = tempfile.mkstemp(dir=path.parent, prefix=path.name + '.', suffix='.tmp')
    try:
        with os.fdopen(fd, 'w') as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def analyze_file(dataset_path, output_dir, *, require_validation=True):
    """Keep failures reviewable; accepted calibration exists only on success."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=False)
    raw = Path(dataset_path).read_bytes()
    try:
        result = fit_dataset(json.loads(raw), require_validation=require_validation)
    except (ValueError, KeyError, TypeError) as error:
        result = {'schema': 'estimator_processed_imu_calibration_v1', 'accepted': False,
                  'failures': [str(error)], 'firmware_applied': False, 'flight_validated': False}
    result['dataset_sha256'] = hashlib.sha256(raw).hexdigest()
    try:
        dataset = json.loads(raw)
    except ValueError:
        dataset = {}
    if 'rotations' in dataset and 'gyro_residual_bias_rad_s' in result:
        from Interaction.estimator_gyro_calibration import fit_gyro
        try:
            gyro = fit_gyro(dataset['rotations'], result['gyro_residual_bias_rad_s'])
            result['gyro_calibration'] = gyro
            result['gyro_axis_scale_calibrated'] = gyro['accepted']
            if not gyro['accepted']:
                raise ValueError('gyro independent rotation error exceeds limit')
        except (ValueError, KeyError, TypeError) as error:
            result['accepted'] = False
            result['fit_passed'] = False
            result['failures'].append('gyro calibration: '+str(error))
    result['dataset_path'] = str(Path(dataset_path).resolve())
    write_json(output_dir / 'report.json', result)
    if result['accepted']:
        write_json(output_dir / 'calibration.json', result)
    if result.get('fit_passed'):
        write_json(output_dir / 'candidate.json', result)
    lines = ['# Estimator IMU calibration', '', f"Accepted: {result['accepted']}",
             '', 'Firmware applied: False. Flight validated: False.', '']
    if result.get('calibration_method') == 'gravity_norm':
        lines += ['Method: gravity magnitude, diagonal axis scale + bias.',
                  'Sensor/body alignment rotation: NOT calibrated.',
                  'Cross-axis terms: NOT calibrated. No attitude reference was used.',
                  f"Axis scales: {result['axis_scales']}",
                  f"Accelerometer bias (m/s²): {result['accel_bias_m_s2']}",
                  f"Residual gyro bias (deg/s): {np.rad2deg(result['gyro_residual_bias_rad_s']).tolist()}", '',
                  '| Pose | Role | Raw norm RMSE (m/s²) | Corrected norm RMSE (m/s²) | Mean norm error (m/s²) |',
                  '| --- | --- | ---: | ---: | ---: |']
        lines += [f"| {name} | {row['role']} | {row['raw_norm_rmse_m_s2']:.4f} | {row['corrected_norm_rmse_m_s2']:.4f} | {row['corrected_norm_mean_error_m_s2']:.4f} |"
                  for name, row in result['pose_metrics'].items()]
    elif 'pose_metrics' in result:
        lines += [f"Alignment rotation: {result['alignment_rotation_deg']:.3f} deg",
                  f"Accelerometer bias (m/s²): {result['accel_bias_m_s2']}",
                  f"Residual gyro bias (deg/s): {np.rad2deg(result['gyro_residual_bias_rad_s']).tolist()}", '',
                  '| Pose | Role | Raw angle (deg) | Corrected angle (deg) | Corrected RMSE (m/s²) |',
                  '| --- | --- | ---: | ---: | ---: |']
        lines += [f"| {name} | {row['role']} | {row['uncorrected_angle_deg']:.3f} | {row['corrected_angle_deg']:.3f} | {row['corrected_rmse_m_s2']:.4f} |"
                  for name, row in result['pose_metrics'].items()]
    lines += ['', *result['failures'], '',
              'Vicon transport delay and gyro axis/scale are not identified by static poses.',
              'Do not use an ordinary-KF or Contact-ESKF quaternion as fixture truth.']
    if 'gyro_calibration' in result:
        gyro = result['gyro_calibration']
        lines += ['', '## Gyro scale and axis calibration', '', f"Accepted: {gyro['accepted']}",
                  f"Alignment rotation: {gyro['alignment_rotation_deg']:.3f} deg", '',
                  '| Turn | Role | Rotation error norm (deg) |', '| --- | --- | ---: |']
        lines += [f"| {name} | {row['role']} | {row['error_norm_deg']:.4f} |"
                  for name, row in gyro['rotation_metrics'].items()]
    (output_dir / 'report.md').write_text('\n'.join(lines) + '\n')
    return result
