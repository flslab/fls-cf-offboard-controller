"""Orientation-free static IMU calibration using only the gravity magnitude.

Fit three positive axis scales and three biases. The correction is diagonal:
gravity magnitude cannot identify the sensor-to-body rotation. NumPy only.
"""
from __future__ import annotations

import numpy as np

METHOD = 'gravity_norm'
REFERENCE = 'gravity_magnitude'
MINIMUM_TRAIN_POSES = 18
MINIMUM_VALIDATION_POSES = 6
MINIMUM_POSE_SEPARATION_DEG = 8.
MAXIMUM_DESIGN_CONDITION = 15.


def gravity_plan():
    """Approximate directions aid coverage, but are NEVER gravity references."""
    regions = ('top roughly up', 'top roughly down', 'front roughly up',
               'front roughly down', 'left side roughly up', 'right side roughly up')
    changes = ('rest the cage securely', 'tilt about 20-40 degrees toward another side',
               'tilt about 20-40 degrees in a different direction')
    plan = []
    for repeat, change in enumerate(changes):
        for index, region in enumerate(regions):
            plan.append({'id': f'train_pose_{repeat*6+index+1:02d}', 'role': 'train',
                         'instruction': f'{region}; {change}. Exact angle is NOT required'})
    for index, region in enumerate(regions):
        plan.append({'id': f'validation_pose_{index+1:02d}', 'role': 'validation',
                     'instruction': f'NEW support/tilt with {region}; choose a different '
                                    'angle from training. Exact angle is NOT required'})
    return plan


def check_pose_diversity(pose, previous):
    """Reject duplicate directions even if their sensor noise differs."""
    direction = np.mean(pose['accel_m_s2'], axis=0)
    norm = np.linalg.norm(direction)
    if not .7*9.81 <= norm <= 1.3*9.81:
        raise ValueError('static acceleration magnitude is outside 0.7-1.3 g')
    for old in previous:
        vector = np.mean(old['accel_m_s2'], axis=0)
        cosine = np.dot(direction, vector)/(norm*np.linalg.norm(vector))
        if cosine > np.cos(np.deg2rad(MINIMUM_POSE_SEPARATION_DEG)):
            raise ValueError(f'pose is too similar to {old["id"]}; tilt the cage by '
                             f'at least {MINIMUM_POSE_SEPARATION_DEG:g} degrees')


def _coverage(means, *, validation=False):
    directions = means/np.linalg.norm(means, axis=1)[:, None]
    axis_limit, eigen_limit = (.4, .1) if validation else (.65, .15)
    if (np.any(np.max(directions, axis=0) < axis_limit)
            or np.any(np.min(directions, axis=0) > -axis_limit)
            or np.linalg.eigvalsh(directions.T@directions/len(directions))[0] < eigen_limit):
        raise ValueError('gravity poses need both signs of X/Y/Z and broad 3D coverage')


def _design(means, gravity):
    _coverage(means)
    u = means/gravity
    design = np.c_[u*u, u]
    condition = float(np.linalg.cond(design))
    if np.linalg.matrix_rank(design) < 6 or condition > MAXIMUM_DESIGN_CONDITION:
        raise ValueError('gravity poses do not constrain bias/scale; add varied tilted poses')
    return u, design, condition


def _fit(means, gravity):
    u, design, condition = _design(means, gravity)
    coefficients = np.linalg.lstsq(design, np.ones(len(u)), rcond=None)[0]
    quadratic, linear = coefficients[:3], coefficients[3:]
    if np.any(quadratic <= 0):
        raise ValueError('gravity fit has nonpositive scale; check static pose coverage')
    center = -linear/(2*quadratic)
    factor = 1+np.dot(quadratic, center*center)
    scale = np.sqrt(factor/quadratic)
    parameters = np.r_[center, np.log(scale)]

    # Refine geometric norm error rather than algebraic ellipsoid error. Every
    # pose has equal weight; no outlier windows or samples are silently removed.
    damping = 1e-6
    for _ in range(60):
        center, scale = parameters[:3], np.exp(parameters[3:])
        w = (u-center)/scale
        norms = np.linalg.norm(w, axis=1)
        if np.any(norms < 1e-6):
            raise ValueError('degenerate gravity fit')
        residual = norms-1
        jacobian = np.c_[-w/(scale*norms[:, None]), -w*w/norms[:, None]]
        gradient = jacobian.T@residual
        if np.linalg.norm(gradient, ord=np.inf) < 1e-10:
            break
        step = np.linalg.solve(jacobian.T@jacobian+damping*np.eye(6), -gradient)
        proposed = parameters+step
        # Do not wander into a different ellipsoid basin on bad data.
        if np.any(np.abs(proposed[:3]) > .2) or np.any(np.abs(proposed[3:]) > .5):
            damping *= 10
            continue
        trial = np.linalg.norm((u-proposed[:3])/np.exp(proposed[3:]), axis=1)-1
        if np.dot(trial, trial) < np.dot(residual, residual):
            parameters = proposed
            damping = max(1e-12, damping/3)
            if np.linalg.norm(step) < 1e-10:
                break
        else:
            damping *= 10
    return np.diag(np.exp(parameters[3:])), parameters[:3]*gravity, condition


def fit_gravity_dataset(document, *, require_validation=True):
    from Interaction.estimator_imu_calibration import G, LIMITS, SCHEMA, validate_pose

    if document.get('schema') != SCHEMA or document.get('status') != 'complete':
        raise ValueError('need a complete gravity-magnitude dataset')
    if (document.get('reference_source') != REFERENCE
            or document.get('sensor_frame') != 'driver_processed_body'
            or document.get('motors_off_confirmed') is not True):
        raise ValueError('gravity calibration requires motors-off processed body IMU')
    for key in ('drone_id', 'firmware_id', 'fixture_id', 'reference_note'):
        if not isinstance(document.get(key), str) or not document[key].strip():
            raise ValueError(f'missing calibration provenance: {key}')
    capture = document.get('capture', {})
    before = capture.get('firmware_parameters_before', {}).get('imu_sensors')
    after = capture.get('firmware_parameters_after', {}).get('imu_sensors')
    if before is not None and after is not None and before != after:
        raise ValueError('IMU sensor parameters changed during collection')
    grouped = {'train': [], 'validation': []}
    seen, previous = set(), []
    for pose in document.get('poses', []):
        if pose.get('id') in seen or pose.get('role') not in grouped:
            raise ValueError('pose IDs must be unique and roles train/validation')
        seen.add(pose['id'])
        _, acc, gyro = validate_pose(pose, known_reference=False)
        check_pose_diversity(pose, previous)
        previous.append(pose)
        grouped[pose['role']].append((pose, acc, gyro))
    if len(grouped['train']) < MINIMUM_TRAIN_POSES:
        raise ValueError(f'gravity calibration needs at least {MINIMUM_TRAIN_POSES} distinct training poses')
    if require_validation and len(grouped['validation']) < MINIMUM_VALIDATION_POSES:
        raise ValueError(f'gravity calibration needs at least {MINIMUM_VALIDATION_POSES} new validation poses')
    if grouped['validation'] and len(grouped['validation']) < MINIMUM_VALIDATION_POSES:
        raise ValueError('gravity validation needs six new poses, not a partial holdout')
    training = grouped['train']
    matrix, bias, condition = _fit(np.array([acc.mean(axis=0) for _, acc, _ in training]), G)
    scales = np.diag(matrix)
    if np.any((scales < .8) | (scales > 1.2)):
        raise ValueError('invalid accelerometer scale; expected 0.8-1.2')
    if np.max(np.abs(bias)) > .5:
        raise ValueError('accelerometer bias exceeds 0.5 m/s²')
    gyro_means = np.array([gyro.mean(axis=0) for _, _, gyro in training])
    bg = gyro_means.mean(axis=0)
    if np.max(np.ptp(gyro_means, axis=0)) > np.deg2rad(.3):
        raise ValueError('gyro zero changes across training poses; warm up and repeat')
    metrics, failures = {}, []
    if grouped['validation']:
        # Holdouts test a frozen model; they need direction coverage, not the
        # rank/conditioning needed to refit six unknown parameters.
        _coverage(np.array([acc.mean(axis=0) for _, acc, _ in grouped['validation']]), validation=True)
    for pose, acc, gyro in training+grouped['validation']:
        error = np.linalg.norm((acc-bias)/scales, axis=1)-G
        row = {'role': pose['role'], 'sample_count': len(acc),
               'raw_norm_rmse_m_s2': float(np.sqrt(np.mean((np.linalg.norm(acc, axis=1)-G)**2))),
               'corrected_norm_rmse_m_s2': float(np.sqrt(np.mean(error**2))),
               'corrected_norm_mean_error_m_s2': float(np.mean(error)),
               'corrected_gyro_mean_deg_s': np.rad2deg(gyro.mean(axis=0)-bg).tolist()}
        metrics[pose['id']] = row
        limit = LIMITS['maximum_fit_rmse_m_s2'] if pose['role'] == 'train' else LIMITS['maximum_validation_rmse_m_s2']
        if row['corrected_norm_rmse_m_s2'] > limit:
            failures.append(f'{pose["id"]}: corrected gravity norm RMSE > {limit}')
        if pose['role'] == 'validation' and max(abs(v) for v in row['corrected_gyro_mean_deg_s']) > .3:
            failures.append(f'{pose["id"]}: residual gyro zero changed after fitting')
    accepted = not failures and bool(grouped['validation'])
    return {'schema': 'estimator_processed_imu_calibration_v1',
            'calibration_method': METHOD, 'reference_source': REFERENCE,
            'accepted': accepted, 'fit_passed': not failures, 'failures': failures,
            'independent_static_validation_passed': accepted,
            'validation_scope': 'gravity magnitude and static gyro zero only; no attitude reference',
            **{key: document[key] for key in ('drone_id','firmware_id','fixture_id','reference_note')},
            'provenance_documented': all(document[key].strip().lower() != 'unknown'
                                        for key in ('firmware_id','fixture_id','reference_note')),
            'sensor_frame': 'driver_processed_body',
            'measured_from_reference': matrix.tolist(), 'accel_bias_m_s2': bias.tolist(),
            'gyro_residual_bias_rad_s': bg.tolist(), 'axis_scales': scales.tolist(),
            'correction': 'acc_corrected = (acc_measured - accel_bias_m_s2) / axis_scales; gyro_corrected = gyro_measured - gyro_residual_bias_rad_s',
            'alignment_rotation_deg': None, 'accel_alignment_calibrated': False,
            'accel_cross_axis_calibrated': False, 'accel_axis_scale_calibrated': True,
            'gravity_design_condition': condition, 'pose_metrics': metrics, 'limits': dict(LIMITS),
            'gyro_axis_scale_calibrated': False, 'vicon_delay_calibrated': False,
            'firmware_applied': False, 'flight_validated': False,
            'note': 'Cage support is allowed. Static gravity norm does not identify sensor/body rotation or gyro scale/axes. Axis correction is diagonal; body alignment is unchanged and uncalibrated. Residual gyro zero is boot/temperature dependent. No onboard estimator loads this file.'}
