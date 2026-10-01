"""Calibrated *closed-loop* attitude model, not an ESC/motor time constant.

No flight commands. Model parameters are volatile and explicitly committed only
after fresh readback. The original wrench/planar calibration file is untouched.
"""
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import zlib

from Interaction.firmware_parameter_confirmation import confirm_firmware_mode_parameters

DEFAULT_PATH = Path(__file__).with_name('attitude_response.json')
PID_NAMES = tuple(f'{group}.{axis}{term}' for group in ('pid_attitude', 'pid_rate')
                  for axis in ('roll', 'pitch') for term in ('_kp', '_ki', '_kd', '_kff')) + (
    'pid_rate.rateFiltEn', 'pid_rate.omxFiltCut', 'pid_rate.omyFiltCut',
    'pid_attitude.attFiltEn', 'pid_attitude.attFiltCut')
FIELDS = {'delay_s': 'Delay', 'wn_rad_s': 'Wn', 'zeta': 'Zeta', 'gain': 'Gain', 'bias_deg': 'Bias'}


def _finite(value, name, lower, upper):
    if (isinstance(value, bool) or not isinstance(value, (float, int))
            or not math.isfinite(value) or not lower <= value <= upper):
        raise ValueError(f'invalid response model {name}')
    return float(value)


def validate_report(report):
    if (not isinstance(report, dict) or report.get('fit_schema_version') != 1
            or report.get('usable') is not True
            or report.get('clock_basis') != 'firmware_timestamp_ms'
            or report.get('attitude_source') not in ('ordinary', 'post_release15')):
        raise ValueError('no usable device-clock closed-loop attitude calibration')
    ranges = {'delay_s': (0., .149999), 'wn_rad_s': (5.000001, 59.999999),
              'zeta': (.200001, 1.999999), 'gain': (.700001, 1.299999), 'bias_deg': (-3., 3.)}
    for axis in ('roll', 'pitch'):
        fit = report.get('axes', {}).get(axis, {})
        if fit.get('model') != 'delayed_second_order' or fit.get('usable') is not True:
            raise ValueError(f'unusable {axis} response calibration')
        for field, bounds in ranges.items():
            _finite(fit.get(field), axis+'.'+field, *bounds)
        for field, bounds in {'r_squared': (.95, 1.), 'normalized_rmse': (0., .15),
                              'rmse_deg': (0., 3.)}.items():
            _finite(fit.get(field), axis+'.'+field, *bounds)
    return report


def _pid_context(values):
    return {k: _finite(values.get(k), k, 0., 10000.) for k in PID_NAMES}


def confirm_pid_context(param, configured):
    expected = {k: float(configured[k] if k in configured else param.get_value(k))
                for k in PID_NAMES}
    expected = _pid_context(expected)
    return confirm_firmware_mode_parameters(param, expected=expected)


def _atomic_save(path, document):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.'+path.name, dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(document, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write('\n')
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def save_report(drone_id, report, pid_values, *, source_log, path=DEFAULT_PATH):
    """Keep failed attempts for diagnosis; never silently reuse an old fit."""
    path = Path(path)
    document = json.loads(path.read_text()) if path.exists() else {'schema_version': 1, 'drones': {}}
    if document.get('schema_version') != 1 or not isinstance(document.get('drones'), dict):
        raise ValueError('unsupported attitude response file schema')
    entry = deepcopy(document['drones'].get(str(drone_id), {}))
    attempt = {'updated_at': datetime.now(timezone.utc).isoformat(),
               'source_log': str(source_log),
               'source_sha256': hashlib.sha256(Path(source_log).read_bytes()).hexdigest(),
               'report': deepcopy(report), 'pid_values': deepcopy(pid_values)}
    try:
        validate_report(report)
        _pid_context(pid_values)
    except ValueError as error:
        attempt.update(accepted=False, reason=str(error))
    else:
        attempt['accepted'] = True
        entry['accepted'] = deepcopy(attempt)
    entry['latest_attempt'] = attempt
    document['drones'][str(drone_id)] = entry
    _atomic_save(path, document)
    return attempt


def load_model(drone_id, *, path=DEFAULT_PATH, pid_values):
    document = json.loads(Path(path).read_text())
    if document.get('schema_version') != 1:
        raise ValueError('unsupported attitude response file schema')
    entry = document.get('drones', {}).get(str(drone_id), {})
    latest = entry.get('latest_attempt', {})
    if latest.get('accepted') is not True or entry.get('accepted') != latest:
        raise ValueError('latest attitude calibration failed or is missing; old fit not activated')
    validate_report(latest['report'])
    expected, current = _pid_context(latest['pid_values']), _pid_context(pid_values)
    if any(not math.isclose(expected[k], current[k], rel_tol=1e-5, abs_tol=1e-6) for k in expected):
        raise ValueError('PID/filter configuration changed since attitude calibration')
    return deepcopy(latest)


def model_parameters(model):
    report = validate_report(model['report'])
    values = {}
    for axis, prefix in (('roll', 'r'), ('pitch', 'p')):
        values.update({'pRelResp.'+prefix+suffix: float(report['axes'][axis][field])
                       for field, suffix in FIELDS.items()})
    values['pRelResp.source'] = 1 if report['attitude_source'] == 'ordinary' else 2
    identity = zlib.crc32(json.dumps(model, sort_keys=True, allow_nan=False).encode()) or 1
    values['pRelResp.id'] = identity
    return values


def upload_model(param, model, *, timeout_s=5.):
    values = model_parameters(model)
    toc = getattr(getattr(param, 'toc', None), 'toc', {})
    required = set(values) | {'pRelResp.runtime', 'pRelResp.commit',
                              'pRelResp.ready', 'pRelResp.activeId'}
    if any(name.split('.')[1] not in toc.get('pRelResp', {}) for name in required):
        raise RuntimeError('firmware lacks calibrated response model; reflash paired firmware')
    # Require an executing runtime, not a specific build/date identifier.
    confirm_firmware_mode_parameters(param, timeout_s=timeout_s,
        expected={'pRelResp.runtime': 1})
    param.set_value('pRelResp.commit', '0')
    confirm_firmware_mode_parameters(param, timeout_s=timeout_s,
        expected={'pRelResp.ready': 0, 'pRelResp.activeId': 0})
    try:
        for name, value in values.items():
            param.set_value(name, str(value))
        confirm_firmware_mode_parameters(param, timeout_s=timeout_s, expected=values)
        param.set_value('pRelResp.commit', str(values['pRelResp.id']))
        expected = dict(values, **{'pRelResp.ready': 1, 'pRelResp.activeId': values['pRelResp.id']})
        confirm_firmware_mode_parameters(param, timeout_s=timeout_s, expected=expected)
    except Exception:
        param.set_value('pRelResp.commit', '0')
        raise
    return expected


def fit_completed_calibration(drone_id, log_path, pid_values, *, path=DEFAULT_PATH):
    from Interaction.calibration_attitude_response import identify_attitude_response_from_log_records
    records = json.loads(Path(log_path).read_text())
    # An aborted flight/failed main calibration is not a successful new fit.
    if not any(r.get('name') == 'Wrench Model Calibration Saved' for r in records):
        raise ValueError('ordinary calibration did not finish saving')
    try:
        report = identify_attitude_response_from_log_records(records, attitude_source='ordinary')
    except (KeyError, ValueError) as error:
        report = {'usable': False, 'error': str(error)}
    return save_report(drone_id, report, pid_values, source_log=log_path, path=path)


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Fit existing calibration logs without flying')
    parser.add_argument('--from-log', type=Path, required=True)
    parser.add_argument('--drone-id', required=True)
    parser.add_argument('--output', type=Path, required=True,
                        help='explicit output; never overwrites the wrench calibration')
    args = parser.parse_args()
    records = json.loads(args.from_log.read_text())
    contexts = [r.get('data', {}).get('pid_values', {}) for r in records
                if r.get('name') == 'Attitude Response Calibration Context']
    # Old logs without a fresh readback remain useful fit evidence, but cannot
    # establish an exact historical PID/filter configuration by inference.
    result = fit_completed_calibration(args.drone_id, args.from_log,
                                      contexts[-1] if contexts else {}, path=args.output)
    print(json.dumps({'accepted': result['accepted'], 'reason': result.get('reason'),
                      'report': result['report'], 'output': str(args.output)}, indent=2))


if __name__ == '__main__':
    main()
