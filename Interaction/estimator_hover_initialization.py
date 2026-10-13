"""Relative roll/pitch initialization from one contact-free hover window.

Fits only an additional X/Y specific-force offset in the statically corrected
body frame. It does not fit yaw, gyro parameters, Z acceleration, or a mounting
rotation. Default-estimator attitude is a relative reference, not physical truth.
The fit is frozen before held-out replay; nothing here writes aircraft parameters.
"""
from bisect import bisect_right
from copy import deepcopy
import math

import numpy as np

from Interaction.estimator_imu_calibration import G
from Interaction.estimator_validation_flight import rotation

SCHEMA = 'estimator_hover_roll_pitch_v1'
SETTLE_S = 2.
DURATION_S = 3.
WINDOW_EVENT = 'hover_roll_pitch_window'


def hover_window(began):
    return dict(name=WINDOW_EVENT, phase='hover_start',
                start_received_s=float(began)+SETTLE_S,
                end_received_s=float(began)+SETTLE_S+DURATION_S,
                settle_s=SETTLE_S, duration_s=DURATION_S,
                reference='default_estimator_2_relative',
                correction_axes=[0, 1], frozen_after_window=True,
                firmware_applied=False)


def _window(rows):
    explicit = [r['data'] for r in rows if r['group']=='event'
                and r['data'].get('name')==WINDOW_EVENT]
    if explicit:
        if len(explicit)!=1:
            raise ValueError('expected one initial hover training window')
        window = deepcopy(explicit[0])
        if window.get('phase')!='hover_start':
            raise ValueError('training is restricted to the initial contact-free hover')
    else:
        initial = [r for r in rows if r['phase']=='hover_start' and r['group']=='imu']
        if not initial:
            raise ValueError('initial hover is missing; no fit from later motion or interaction')
        window = hover_window(min(r['received_s'] for r in initial))
    start, end = window['start_received_s'], window['end_received_s']
    if not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)
               for v in (start,end)) or abs(end-start-DURATION_S)>.001:
        raise ValueError('hover training window must span exactly 3 seconds')
    return window


def _coverage(rows, minimum, label):
    if len(rows)<minimum:
        raise ValueError(f'{label}: too few hover samples')
    times = np.asarray([r['received_s'] for r in rows],float)
    if not np.isfinite(times).all() or times[-1]-times[0]<2.8:
        raise ValueError(f'{label}: insufficient hover duration')
    if np.any(np.diff(times)<=0) or np.max(np.diff(times))>.05:
        raise ValueError(f'{label}: missing or reordered hover telemetry')
    return times


def fit_hover_roll_pitch(rows, candidate, candidate_sha256):
    """Return a separate, rejectable relative fit; never alter the static fit."""
    report = dict(schema=SCHEMA, accepted=False, failures=[],
                  reference='default_estimator_2_relative', independent_ground_truth=False,
                  source_candidate_sha256=candidate_sha256, correction_axes=[0,1],
                  gyro_changed=False, yaw_calibrated=False, z_acceleration_changed=False,
                  firmware_applied=False, frozen_after_window=True)
    try:
        ordered = sorted(rows,key=lambda r:r['received_s'])
        window = _window(ordered)
        report['window'] = window
        start, end = window['start_received_s'],window['end_received_s']
        training = [r for r in ordered if start<=r['received_s']<end]
        if any(r['phase']!='hover_start' for r in training if r['group']!='metadata'):
            raise ValueError('training window extends beyond the initial hover')
        imu = [r for r in training if r['group']=='imu']
        vicon = [r for r in training if r['group']=='vicon']
        times = _coverage(imu,200,'IMU')
        pt = _coverage(vicon,200,'Vicon')
        ticks = [r['cf_log_tick_ms_mod24'] for r in imu]
        if any(type(t)!=int or not 0<=t<(1<<24) for t in ticks):
            raise ValueError('invalid hover device timestamp')
        gaps = np.asarray([(b-a)%(1<<24) for a,b in zip(ticks,ticks[1:])])
        if np.any(gaps<=0) or np.any(gaps>50):
            raise ValueError('missing/reordered IMU device samples in hover')
        if abs(float(gaps.sum())/1000.-(times[-1]-times[0]))>.05:
            raise ValueError('host/device hover duration disagreement')

        commands = [r['data'] for r in training if r['group']=='command']
        if not commands or any(c.get('estimator')!=2 for c in commands):
            raise ValueError('hover requires position hold controlled by estimator 2')
        targets = np.asarray([c['position_m'] for c in commands],float)
        if targets.shape!=(len(commands),3) or not np.isfinite(targets).all() or np.max(np.ptp(targets,axis=0))>.001:
            raise ValueError('hover position target changed during fitting')
        position = np.asarray([r['data']['position_m'] for r in vicon],float)
        if position.shape!=(len(vicon),3) or not np.isfinite(position).all():
            raise ValueError('invalid hover Vicon positions')
        if np.max(np.ptp(position,axis=0))>.08 or np.max(np.linalg.norm(position-targets[0],axis=1))>.10:
            raise ValueError('hover position is not settled')
        # Account for measured residual motion instead of assuming a perfect hover.
        t = pt-pt[0]
        coefficients = np.stack([np.polyfit(t,position[:,i],2) for i in range(3)])
        acceleration = 2*coefficients[:,0]
        speeds = np.stack([coefficients[:,1],coefficients[:,1]+acceleration*t[-1]])
        fitted_position = np.stack([np.polyval(c,t) for c in coefficients],axis=1)
        position_rmse = float(np.sqrt(np.mean((position-fitted_position)**2)))
        if np.max(np.linalg.norm(speeds,axis=1))>.08 or np.linalg.norm(acceleration)>.15 or position_rmse>.01:
            raise ValueError('hover motion is too large or inconsistent for initialization')

        A = np.asarray(candidate['measured_from_reference'],float)
        b = np.asarray(candidate['accel_bias_m_s2'],float)
        if A.shape!=(3,3) or b.shape!=(3,) or not np.isfinite(A).all() or not np.isfinite(b).all():
            raise ValueError('invalid static accelerometer fit')
        history = [r for r in ordered if r['group']=='kf']
        stamps = [r['received_s'] for r in history]
        residuals, rotations = [], []
        for sample in imu:
            j = bisect_right(stamps,sample['received_s'])-1
            reference = None
            while j>=0 and sample['received_s']-history[j]['received_s']<=.06:
                row = history[j]
                tick = row.get('cf_log_tick_ms_mod24')
                if type(tick)==int and (sample['cf_log_tick_ms_mod24']-tick)%(1<<24)<=60:
                    reference=row
                    break
                j-=1
            if reference is None:
                raise ValueError('no causal fresh default-estimator attitude during hover')
            R = rotation([reference['data'][f'kalman.q{k}'] for k in range(4)])
            acc = np.asarray([sample['data'][f'acc.{k}']*G for k in 'xyz'],float)
            if not np.isfinite(acc).all():
                raise ValueError('nonfinite hover acceleration')
            residuals.append(np.linalg.solve(A,acc-b)-R.T@(acceleration+[0,0,G]))
            rotations.append(R)
        rotations=np.asarray(rotations)
        angle = np.arccos(np.clip((np.trace(rotations[0].T@rotations,axis1=1,axis2=2)-1)/2,-1,1))
        if np.rad2deg(angle).max()>3.:
            raise ValueError('hover attitude changed by more than 3 degrees')
        residuals=np.asarray(residuals)
        first,last=np.array_split(residuals,2)
        if np.max(np.abs(first[:,:2].mean(axis=0)-last[:,:2].mean(axis=0)))>.10:
            raise ValueError('hover X/Y residual is not stable')
        delta=residuals.mean(axis=0)
        delta[2]=0.  # No Z correction or yaw/gyro fitting in this stage.
        if np.max(np.abs(delta))>.5:
            raise ValueError('hover X/Y residual exceeds 0.5 m/s²')
        report.update(accepted=True, training_samples=len(imu),
                      additional_body_accel_bias_m_s2=delta.tolist(),
                      vicon_acceleration_m_s2=acceleration.tolist(),
                      vicon_position_fit_rmse_m=position_rmse,
                      max_reference_rotation_deg=float(np.rad2deg(angle).max()),
                      residual_half_window_change_xy_m_s2=(first[:,:2].mean(axis=0)-last[:,:2].mean(axis=0)).tolist())
    except (ValueError,KeyError,TypeError,np.linalg.LinAlgError) as error:
        report['failures'].append(str(error))
    return report


def apply_hover_roll_pitch(candidate, report, candidate_sha256):
    """Create an offline-only effective candidate; source is unchanged."""
    delta=np.asarray(report.get('additional_body_accel_bias_m_s2'),float)
    if (report.get('schema')!=SCHEMA or report.get('accepted') is not True
            or report.get('source_candidate_sha256')!=candidate_sha256
            or report.get('correction_axes')!=[0,1] or report.get('firmware_applied') is not False
            or report.get('gyro_changed') is not False or report.get('yaw_calibrated') is not False
            or report.get('z_acceleration_changed') is not False or report.get('frozen_after_window') is not True
            or delta.shape!=(3,) or not np.isfinite(delta).all() or delta[2]!=0.
            or np.max(np.abs(delta))>.5):
        raise ValueError('invalid or mismatched hover roll/pitch initialization')
    effective=deepcopy(candidate)
    effective['accel_bias_m_s2']=(np.asarray(candidate['accel_bias_m_s2'])+
        np.asarray(candidate['measured_from_reference'])@delta).tolist()
    return effective


def held_out_hover_rows(rows, report):
    """Both comparison arms seed once at the same epoch after training ends."""
    if report.get('accepted') is not True:
        raise ValueError('hover initialization was rejected')
    end=report['window']['end_received_s']
    selected=[r for r in rows if r['group']!='event' and r['received_s']>=end-.06]
    selected.append(dict(group='event', received_s=end, phase='hover_start',
                         cf_log_tick_ms_mod24=None, data={'name':'capture_ready'}))
    return sorted(selected,key=lambda r:r['received_s'])
