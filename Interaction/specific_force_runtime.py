"""Configuration and raw-IMU transport for the opt-in model force estimator."""
from copy import deepcopy
import math
import numpy as np

IMU_GROUP = 'MODEL_IMU'
IMU_FIELDS = tuple(f'{sensor}.{axis}' for sensor in ('acc', 'gyro') for axis in 'xyz')
DEFAULTS = dict(window_s=.08, minimum_window_s=.05, max_dt_s=.05,
                max_pair_skew_s=.009, max_sample_age_s=.1,
                accel_scale=[1., 1., 1.], accel_bias_m_s2=[0., 0., 0.],
                imu_from_com_m=[0., 0., 0.], thrust_scale=1.)


def force_estimator(config):
    method = config.get('force_estimator', 'momentum')
    if method not in ('momentum', 'specific_force'):
        raise ValueError('force_estimator must be momentum or specific_force')
    return method


def validate_config(config):
    result = deepcopy(DEFAULTS)
    overrides = config.get('specific_force') or {}
    if not isinstance(overrides, dict) or set(overrides) - set(result):
        raise ValueError('specific_force has unknown settings or is not a mapping')
    result.update(overrides)
    for key in ('window_s', 'minimum_window_s', 'max_dt_s', 'max_pair_skew_s',
                'max_sample_age_s', 'thrust_scale'):
        value = result[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'specific_force.{key} must be finite and positive')
    if result['minimum_window_s'] > result['window_s']:
        raise ValueError('specific_force window must cover minimum_window_s')
    if result['max_pair_skew_s'] >= .01:
        raise ValueError('specific_force pairing must be tighter than one 10 ms log cycle')
    for key in ('accel_scale', 'accel_bias_m_s2', 'imu_from_com_m'):
        value = np.asarray(result[key], dtype=float)
        if value.shape != (3,) or not np.isfinite(value).all():
            raise ValueError(f'specific_force.{key} must contain three finite values')
        if key == 'accel_scale' and np.any(value <= 0):
            raise ValueError('specific_force.accel_scale must be positive')
        result[key] = value.tolist()
    return result


def requested(mission):
    config = (mission or {}).get('Interaction', {}).get('config', {})
    if config.get('detection_method', 'potentiometer') != 'model':
        return False
    wrench = config.get('wrench_interaction') or {}
    if force_estimator(wrench) != 'specific_force':
        return False
    if (config.get('behavior') != 'level_coast'
            or wrench.get('state_source') != 'onboard'):
        raise ValueError('specific_force requires level_coast, model detection and onboard state')
    validate_config(wrench)
    return True


def add_imu_logs(selected, mission):
    if not requested(mission):
        return selected
    return {**selected, IMU_GROUP: {
        **{name: {'type': 'float', 'data': []} for name in IMU_FIELDS},
        'log_period_ms': 10,
    }}


class SpecificForceTelemetry:
    """Pair on device time; unwrap IMU time; never reuse a sample for dwell."""
    def __init__(self, config):
        self.config = validate_config(config)
        self.last_tick = None
        self.clock_s = 0.
        self.diagnostics = {}

    def read(self, logs, state, now):
        tick = state.get('cf_timestamp_ms')
        motor_skew = state.get('motor_skew_s')
        if (tick is None or motor_skew is None or not math.isfinite(motor_skew)
                or not 0 <= motor_skew <= self.config['max_pair_skew_s']):
            return None
        packet, skew = logs.get_nearest_group_log_data_by_cf_timestamp(IMU_GROUP, tick)
        if packet is None or skew is None or not math.isfinite(skew) or not 0 <= skew <= self.config['max_pair_skew_s']:
            return None
        metadata, _ = logs.get_nearest_group_log_metadata(IMU_GROUP, packet['time'])
        if (metadata is None or metadata.get('cf_timestamp_ms') is None
                or metadata.get('time') != packet['time']):
            return None
        age = now - packet['time']
        if not math.isfinite(age) or not 0 <= age <= self.config['max_sample_age_s']:
            return None
        imu_tick = int(metadata['cf_timestamp_ms']) & 0xFFFFFF
        delta = None if self.last_tick is None else ((imu_tick-self.last_tick+0x800000)&0xFFFFFF)-0x800000
        if delta == 0:
            return None
        if delta is not None and delta < 0:
            raise RuntimeError('specific_force IMU device clock moved backwards')
        acc = np.array([packet['acc.'+a] for a in 'xyz'], float)*9.81
        gyro = np.deg2rad([packet['gyro.'+a] for a in 'xyz'])
        if not np.isfinite(np.r_[acc, gyro]).all():
            raise RuntimeError('specific_force raw IMU contains nonfinite data')
        self.clock_s += 0. if delta is None else delta/1000.
        self.last_tick = imu_tick
        self.diagnostics = dict(imu_cf_timestamp_ms=imu_tick, imu_age_s=age,
                                imu_state_skew_s=skew, imu_sample_time_s=self.clock_s)
        return dict(specific_force_body_m_s2=acc, gyro_body_rad_s=gyro,
                    imu_timestamp=self.clock_s)
