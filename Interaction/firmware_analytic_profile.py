"""Explicit, read-back-confirmed selection of bounded onboard brake paths.

This module only validates configuration. It sends no packets or flight commands.
The controller selects the bounded runtime when this profile is present;
`execution` must agree with the mission's user-visible `command_mode`.
"""
import math


def profile_parameters(config):
    if config is None:
        return {}
    if not isinstance(config, dict):
        raise ValueError('analytic_profile must be a mapping')
    allowed = {'shape', 'execution', 'tail_s', 'single_s', 'handoff', 'feedback',
               'attitude_kp', 'attitude_kv', 'rate_feedforward'}
    if set(config) - allowed:
        raise ValueError('unknown analytic_profile fields: ' + ', '.join(sorted(set(config) - allowed)))
    def choice(key, choices):
        value = config.get(key)
        if not isinstance(value, str) or value not in choices:
            raise ValueError(f'analytic_profile.{key} must be one of {tuple(choices)}')
        return choices[value]
    def number(key, lo, hi, default=None):
        value = config.get(key, default)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lo <= value <= hi:
            raise ValueError(f'analytic_profile.{key} must be finite in [{lo}, {hi}]')
        return float(value)
    rate_ff=config.get('rate_feedforward', False)
    if type(rate_ff) is not bool:
        raise ValueError('analytic_profile.rate_feedforward must be boolean')
    return {
        'hlCommander.pRelLite': 0,
        'hlCommander.pRelVelCmd': 1,
        'hlCommander.pRelAdapt': 0,
        'hlCommander.pRelShape': choice('shape', {'velocity_scurve': 0, 'single_position_polynomial': 1}),
        'hlCommander.pRelExec': choice('execution', {'velocity': 0, 'position': 1, 'attitude': 2}),
        'hlCommander.pRelTail': number('tail_s', .15, 3),
        'hlCommander.pRelOneT': number('single_s', .15, 4),
        'hlCommander.pRelAttP': number('attitude_kp', 0, 3, .8),
        'hlCommander.pRelAttV': number('attitude_kv', .1, 6, 2),
        'hlCommander.pRelAttFF': int(rate_ff),
        'hlCommander.pRelHold': choice('handoff', {'current_position': 0, 'predicted_position': 1, 'predicted_bumpless': 2}),
        'kalmanPRel.feedback': choice('feedback', {'roll_pitch_only': 1, 'unified_vicon15': 2}),
        'kalmanPRel.fuseVicon': 1,
        'kalmanPRel.viconCI': .999,
    }
