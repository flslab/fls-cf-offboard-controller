"""Gyro affine calibration from independently known, axis-constrained turns.

Integrals are valid only for a fixed mechanical rotation axis. Arbitrary hand
rotation is not a known angular-rate reference and is explicitly unsupported.
"""
import numpy as np


def rotation_plan():
    return [{'id': f'gyro_{role}_{axis}{label}', 'role': role,
             'instruction': f'Rotate BODY {axis.upper()} {label}90 degrees between checked mechanical stops',
             'reference_rotation_rad': [sign*np.pi/2 if i == j else 0. for i in range(3)],
             'reference_source': 'axis_constrained_fixture'}
            for role in ('train', 'validation') for j, axis in enumerate('xyz')
            for sign, label in ((1, '+'), (-1, '-'))]


def rotation_integral(window, bias):
    t = np.asarray(window['time_s'], float)
    w = np.asarray(window['gyro_rad_s'], float)
    reference = np.asarray(window['reference_rotation_rad'], float)
    if (t.ndim != 1 or len(t) < 600 or w.shape != (len(t), 3)
            or not np.isfinite(t).all() or not np.isfinite(w).all()
            or np.any(np.diff(t) <= 0) or np.max(np.diff(t)) > .05+1e-9
            or t[-1]-t[0] < 9.):
        raise ValueError('gyro turn needs >=600 samples, >=9s, strictly increasing ticks and gaps <=50ms')
    if (reference.shape != (3,) or not np.isfinite(reference).all()
            or np.count_nonzero(np.abs(reference) > 1e-9) != 1
            or not np.pi/4 <= np.linalg.norm(reference) <= np.pi
            or window.get('reference_source') != 'axis_constrained_fixture'):
        raise ValueError('gyro reference must be a checked 45–180 degree fixed-axis fixture turn')
    corrected = w-np.asarray(bias)
    for region in (t <= t[0]+1.5, t >= t[-1]-1.5):
        still = corrected[region]
        if (len(still) < 50 or np.max(np.abs(np.mean(still, axis=0))) > np.deg2rad(.3)
                or np.max(np.std(still, axis=0)) > np.deg2rad(.8)):
            raise ValueError('gyro turn must start/end stationary; zero changed or movement crossed window boundary')
    if np.max(np.linalg.norm(corrected, axis=1)) > np.deg2rad(120):
        raise ValueError('rotate slowly: maximum 120 deg/s')
    if np.max(np.linalg.norm(corrected, axis=1)) < np.deg2rad(5):
        raise ValueError('no usable gyro rotation was recorded')
    measured = np.sum((corrected[1:]+corrected[:-1])*.5*np.diff(t)[:, None], axis=0)
    return reference, measured


def fit_gyro(windows, bias):
    rows = {'train': [], 'validation': []}
    identifiers = set()
    fingerprints = set()
    import hashlib
    for window in windows:
        if window.get('role') not in rows or window['id'] in identifiers:
            raise ValueError('gyro windows need unique IDs and train/validation roles')
        identifiers.add(window['id'])
        ref, measured = rotation_integral(window, bias)
        fingerprint = hashlib.sha256(np.asarray(window['gyro_rad_s'], float).tobytes()).hexdigest()
        if fingerprint in fingerprints:
            raise ValueError('gyro validation requires independently collected turns')
        fingerprints.add(fingerprint)
        rows[window['role']].append((window['id'], ref, measured))
    for role, group in rows.items():
        ref = np.array([r[1] for r in group])
        if (len(group) < 6 or np.linalg.matrix_rank(ref) != 3 or np.linalg.cond(ref) > 5
                or np.any(np.max(ref, axis=0) < np.pi/4) or np.any(np.min(ref, axis=0) > -np.pi/4)):
            raise ValueError(f'gyro {role} needs positive/negative turns about all three body axes')
    train = rows['train']
    matrix = np.linalg.lstsq(np.array([r[1] for r in train]), np.array([r[2] for r in train]), rcond=None)[0].T
    u, s, vt = np.linalg.svd(matrix)
    angle = np.rad2deg(np.arccos(np.clip((np.trace(u@vt)-1)/2, -1, 1)))
    if np.linalg.det(matrix) <= 0 or np.any((s < .8) | (s > 1.2)) or angle > 10:
        raise ValueError('implausible gyro scale/alignment; check units, axis labels and mechanical stops')
    metrics = {}
    for role, group in rows.items():
        for name, ref, measured in group:
            error = np.rad2deg(np.linalg.solve(matrix, measured)-ref)
            metrics[name] = {'role': role, 'rotation_error_deg': error.tolist(),
                             'error_norm_deg': float(np.linalg.norm(error))}
    passed = all(row['error_norm_deg'] <= (1. if row['role'] == 'train' else 2.) for row in metrics.values())
    return {'schema': 'estimator_processed_gyro_calibration_v1', 'accepted': passed,
            'measured_from_reference': matrix.tolist(), 'inverse_matrix': np.linalg.inv(matrix).tolist(),
            'bias_rad_s': np.asarray(bias).tolist(), 'alignment_rotation_deg': float(angle),
            'singular_values': s.tolist(), 'rotation_metrics': metrics,
            'limits': {'training_angle_error_deg': 1., 'validation_angle_error_deg': 2.},
            'reference_source': 'axis_constrained_fixture',
            'correction': 'gyro_corrected = solve(measured_from_reference, gyro_measured - bias_rad_s)',
            'note': 'Requires a fixed axis and independently checked angle stops, not freehand rotations or estimator attitude as truth.'}
