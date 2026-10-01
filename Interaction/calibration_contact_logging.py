"""Capture-only telemetry for a contact-free, ordinary XYZ calibration.

No observer, detector, firmware parameter, or flight command is changed here.
Raw acc is body specific force in g, not world acceleration with gravity removed.
"""

from copy import deepcopy
import hashlib
import json
import logging
from pathlib import Path

from Interaction.config import ATT_DES
from Interaction.wrench_model_calibration import DEFAULT_CALIBRATION_PATH

logger = logging.getLogger(__name__)

FORCE_IMU = {
    'log_period_ms': 10,
    **{f'{sensor}.{axis}': {'type': 'float', 'unit': unit}
       for sensor, unit in (('acc', 'g'), ('gyro', 'deg/s'))
       for axis in 'xyz'},
}
_WIRE_BYTES = {
    'uint8_t': 1, 'int8_t': 1, 'uint16_t': 2, 'int16_t': 2,
    'FP16': 2, 'uint32_t': 4, 'int32_t': 4, 'float': 4,
}
_STATE_GROUPS = {
    'FIRMWARE_KIN', 'FIRMWARE_ACT', 'VEL_ORI', 'POS_ACC', 'RATE_EST',
    'MOT_BAT', 'ATT_DES', 'ATT_RATE_CTL', 'POS_VEL_CTL',
}


def is_plain_xyz_calibration(args):
    return bool(getattr(args, 'calibrate', False) and not any(
        getattr(args, name, False) for name in (
            'interaction', 'braking_test', 'targeted_braking_calibration',
            'adaptive_braking_calibration', 'planar_braking_calibration',
            'mpc', 'ground_test', 'droneless',
        )
    ))


def calibration_log_vars(selected):
    """Add one 24-byte IMU block; retain existing control-consumed groups."""
    result = deepcopy(selected)
    result['FORCE_IMU'] = deepcopy(FORCE_IMU)
    fields = {key for group in result.values() for key in group}
    if not {'controller.roll', 'controller.pitch'}.issubset(fields):
        result['ATT_DES'] = deepcopy(ATT_DES)
    for name in _STATE_GROUPS & result.keys():
        result[name]['log_period_ms'] = 10
    return result


def _calibration_snapshot(mission, drone_id):
    config = (mission.get('Interaction') or {}).get('config') or {}
    path = Path(config.get('wrench_calibration_file', DEFAULT_CALIBRATION_PATH))
    snapshot = {'path': str(path.resolve()), 'drone_id': drone_id}
    try:
        contents = path.read_bytes()
    except FileNotFoundError:
        return {**snapshot, 'status': 'missing', 'entry': None}
    document = json.loads(contents)
    entry = document.get('drones', {}).get(str(drone_id))
    return {**snapshot, 'status': 'saved' if entry is not None else 'drone_missing',
            'sha256': hashlib.sha256(contents).hexdigest(), 'entry': entry}


def capture_group_manifest(cf, selected, default_period_ms, *, label='calibration'):
    """Check a read-only capture's existing subscriptions before starting them."""
    # Firmware supports 16 blocks; reserve one for controller battery polling.
    if len(selected) > 15:
        raise ValueError(f'{label} capture exceeds the 15-block log budget')
    toc = cf.log.toc.toc
    groups = {}
    missing = []
    for name, group in selected.items():
        variables = {key: value for key, value in group.items()
                     if key != 'log_period_ms'}
        payload = sum(_WIRE_BYTES[value['type']] for value in variables.values())
        if payload > 26:
            raise ValueError(f'{label} log block {name} exceeds 26 bytes')
        for field in variables:
            prefix, variable = field.split('.', 1)
            if variable not in toc.get(prefix, {}):
                missing.append(field)
        groups[name] = {
            'requested_period_ms': group.get('log_period_ms', default_period_ms),
            'payload_bytes': payload,
            'variables': {key: {k: v for k, v in value.items() if k != 'data'}
                          for key, value in variables.items()},
        }
    if missing:
        raise RuntimeError(f'{label} capture needs missing firmware log variables: '
                           + ', '.join(sorted(set(missing))))
    # Log operations are per variable subscription, including duplicates.
    variable_count = sum(len(group['variables']) for group in groups.values())
    if variable_count > 127:
        raise ValueError(f'{label} capture exceeds the 127-variable log budget')
    return groups, variable_count


def configure_calibration_capture(log_manager, cf, selected, mission, args):
    """Validate subscriptions and save the pre-fit model before takeoff."""
    selected = calibration_log_vars(selected)
    groups, variable_count = capture_group_manifest(cf, selected, args.cf_log_period)
    manifest = {
        'schema_version': 1,
        'capture_only': True,
        'protocol': 'ordinary_xyz_calibration',
        'no_contact_is_operator_requirement_not_measured_ground_truth': True,
        'live_contact_decisions_disabled': True,
        'imu_requested_rate_hz': 100,
        'imu_timestamp_basis': 'CRTP log tick, not individual sensor sampling time',
        'imu_atomic_sensor_snapshot': False,
        'cf_timestamp_modulus_ms': 1 << 24,
        'cflib_period_scale': 1 if getattr(args, 'crazysim', False) else 10,
        'groups': groups,
        'requested_packets_per_s': sum(1000.0 / group['requested_period_ms']
                                      for group in groups.values()),
        'variable_subscriptions': variable_count,
        'mission_before_calibration': deepcopy(mission),
        'calibration_before_flight': _calibration_snapshot(mission, args.drone_id),
        'evaluation_note': (
            'Replay with detector arming enabled offline. Evaluate the frozen pre-flight '
            'model separately from any model fitted on this flight. Check packet gaps '
            'and actual rates; zero live calibration onsets is not a false-positive result.'
        ),
    }
    log_manager.live_logger.write({'type': 'calibration_contact_capture', 'data': manifest})
    log_manager.capture_packet_timing = True
    logger.info('Calibration contact capture enabled: raw acc/gyro requested at 100 Hz; '
                '%d log blocks. Capture only; fly without hand contact. '
                'False positives will be evaluated offline.', len(selected))
    return selected
