"""Explicit, read-back-confirmed selection of bounded onboard brake paths.

This module only validates configuration. It sends no packets or flight commands.
The controller selects the bounded runtime when this profile is present;
`execution` must agree with the mission's user-visible `command_mode`.
"""
import math

LEGACY_TILT_LIMIT_DEG = math.degrees(math.atan(4.0 / 9.81))
MAX_TILT_LIMIT_DEG = 40.0
DEFAULT_SCURVE_TILT_LIMIT_DEG = 40.0
# Rounded down from g*tan(40 degrees); a reference limit, not measured braking.
MAX_STOP_DECELERATION_M_S2 = 8.23


def default_tilt_limit(profile):
    profile = profile or {}
    if (profile.get('shape') == 'velocity_scurve'
            and profile.get('execution') == 'attitude'
            and not profile.get('state_matched_start', False)):
        return DEFAULT_SCURVE_TILT_LIMIT_DEG
    return LEGACY_TILT_LIMIT_DEG


def profile_parameters(config):
    if config is None:
        return {}
    if not isinstance(config, dict):
        raise ValueError('analytic_profile must be a mapping')
    allowed = {'shape', 'execution', 'tail_s', 'single_s', 'handoff', 'feedback',
               'attitude_kp', 'attitude_kv', 'rate_feedforward',
               'response_compensation', 'response_bandwidth', 'acceleration_residual',
               'state_matched_start', 'position_tracking_bandwidth', 'friction_from_interaction',
               'max_tilt_deg', 'marker_from_imu_m', 'auto_unwind', 'end_velocity_m_s'}
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
    compensate=config.get('response_compensation', False)
    if type(compensate) is not bool:
        raise ValueError('analytic_profile.response_compensation must be boolean')
    if compensate and (config.get('execution') != 'attitude'
                       or config.get('shape') != 'velocity_scurve' or rate_ff):
        raise ValueError('response_compensation requires velocity_scurve attitude execution '
                         'and rate_feedforward: false (calibrated ordinary attitude model)')
    if 'response_bandwidth' in config and not compensate:
        raise ValueError('response_bandwidth requires response_compensation')
    if 'position_tracking_bandwidth' in config and not compensate:
        raise ValueError('position_tracking_bandwidth requires response_compensation')
    residual=config.get('acceleration_residual', False)
    if type(residual) is not bool:
        raise ValueError('analytic_profile.acceleration_residual must be boolean')
    if residual and not compensate:
        raise ValueError('acceleration_residual requires response_compensation')
    state_match=config.get('state_matched_start', False)
    if type(state_match) is not bool:
        raise ValueError('analytic_profile.state_matched_start must be boolean')
    if state_match and not compensate:
        raise ValueError('state_matched_start requires response_compensation')
    friction = config.get('friction_from_interaction', False)
    if type(friction) is not bool:
        raise ValueError('friction_from_interaction must be boolean')
    if friction and (not compensate or state_match):
        raise ValueError('friction_from_interaction requires compensated velocity_scurve attitude execution and state_matched_start: false')
    automatic=config.get('auto_unwind',False)
    if type(automatic) is not bool:
        raise ValueError('analytic_profile.auto_unwind must be boolean')
    if automatic and (not compensate or state_match or friction or
                      config.get('feedback')!='unified_vicon15' or
                      config.get('position_tracking_bandwidth',0)!=0):
        raise ValueError('auto_unwind requires compensated unified_vicon15 attitude execution, '
                         'state_matched_start: false, no friction/position tracking')
    if 'end_velocity_m_s' in config and not automatic:
        raise ValueError('end_velocity_m_s requires auto_unwind')
    result = {
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
        'hlCommander.pRelHold': choice('handoff', {'current_position': 0, 'predicted_position': 1, 'predicted_bumpless': 2,
                                                'curve_endpoint_forward': 3}),
        'kalmanPRel.feedback': choice('feedback', {'roll_pitch_only': 1, 'unified_vicon15': 2}),
        'kalmanPRel.fuseVicon': 1,
        'kalmanPRel.viconCI': .999,
    }
    if 'response_compensation' in config:
        result['hlCommander.pRelComp'] = int(compensate)
    if compensate:
        result['hlCommander.pRelCompW'] = number('response_bandwidth', 6, 16, 12)
    if 'position_tracking_bandwidth' in config:
        # Optional fourth tracking-error pole (1/s); 0 preserves the stable
        # velocity/acceleration/jerk-only branch. No curve replanning enabled.
        result['hlCommander.pRelCompP'] = number('position_tracking_bandwidth', 0, 6)
    if 'acceleration_residual' in config:
        result['hlCommander.pRelCompB'] = int(residual)
    if 'state_matched_start' in config:
        result['hlCommander.pRelSeed'] = int(state_match)
    if 'friction_from_interaction' in config:
        result['hlCommander.pRelFric'] = int(friction)
    if 'max_tilt_deg' in config:
        if config.get('shape') != 'velocity_scurve' or config.get('execution') != 'attitude':
            raise ValueError('max_tilt_deg requires velocity_scurve attitude execution')
        if state_match:
            raise ValueError('max_tilt_deg requires state_matched_start: false')
        result['hlCommander.pRelTilt'] = number('max_tilt_deg', .1, MAX_TILT_LIMIT_DEG)
    if 'marker_from_imu_m' in config:
        from Interaction.marker_geometry import parameters
        result.update(parameters(config['marker_from_imu_m']))
    if 'auto_unwind' in config:
        result['hlCommander.pRelUnwAuto']=int(automatic)
    if automatic:
        result['hlCommander.pRelUnwVer']=1
        result['hlCommander.pRelEndV']=number('end_velocity_m_s',0,.02,0)
    return result
