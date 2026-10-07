"""Opt-in, read-only roll telemetry from the existing S-curve firmware.

Quaternion packets retain the ordinary KF and contact ESKF simultaneously.
CRTP ticks align packets; contact timeUs identifies the ESKF publication epoch.
These are estimator outputs, not independent attitude ground truth.
"""

from copy import deepcopy


def _block(names):
    return {'log_period_ms': 20,
            **{name: {'type': 'float', 'data': []} for name in names}}


ROLL_DIAGNOSTIC_LOGS = {
    'ROLL_KF': _block([f'kalman.q{i}' for i in range(4)]),
    'ROLL_CONTACT': {
        **_block([f'kalmanPRel.q{i}' for i in range(4)]),
        'kalmanPRel.valid': {'type': 'uint8_t', 'data': []},
        'kalmanPRel.active': {'type': 'uint8_t', 'data': []},
        'kalmanPRel.timeUs': {'type': 'uint32_t', 'data': []},
    },
    'ROLL_ATT_PID': _block([
        'controller.roll', 'controller.rollRate',
        *[f'pid_attitude.roll_out{term}' for term in ('P', 'I', 'D', 'FF')],
    ]),
    'ROLL_RATE_PID': {
        **_block(['controller.cmd_roll',
                  *[f'pid_rate.roll_out{term}' for term in ('P', 'I', 'D', 'FF')]]),
        'controller.resetXY': {'type': 'uint32_t', 'data': []},
    },
}


def add_roll_diagnostics(selected, mission):
    config = (mission.get('Interaction') or {}).get('config') or {}
    options = config.get('level_coast') or {}
    enabled = options.get('roll_diagnostics', False)
    if type(enabled) is not bool:
        raise ValueError('level_coast.roll_diagnostics must be boolean')
    if not enabled:
        return selected
    if (config.get('behavior') != 'level_coast'
            or options.get('coast_command_mode') != 'scurve'):
        raise ValueError('roll_diagnostics requires level_coast with scurve coasting')
    # Existing brake/control groups retain their original rates and fields.
    return {**selected, **deepcopy(ROLL_DIAGNOSTIC_LOGS)}
