"""Pre-arm admission for an explicit hardware seventh-order braking test."""

from __future__ import annotations

import math

from Interaction.interactions import apply_required_jerk_braking_calibration
from Interaction.wrench_model_calibration import DEFAULT_CALIBRATION_PATH


def validate_active_septic_brake_test(
        mission, *, drone_id, sense_axis, sense_sign):
    """Check the exact saved plant fit before any flight command is issued.

    This is an additional early gate, not a substitute for the runtime gate in
    ``InteractionsControl``. Neither check may grant authority from YAML alone.
    """
    interaction = mission.get('Interaction', {})
    if interaction.get('action') != 'translation':
        raise ValueError('active septic brake test requires translation action')
    config = interaction.get('config', {})
    if config.get('detection_method') != 'momentum_impulse':
        raise ValueError('active septic brake test requires momentum_impulse')
    virtual = config.get('virtual_object', {})
    if virtual.get('contact_detection', {}).get('source') != 'potentiometer':
        raise ValueError('active septic brake test requires potentiometer contact')
    if virtual.get('release_behavior', {}).get('mode') != 'potentiometer_coast':
        raise ValueError('active septic brake test requires potentiometer_coast')
    wrench = config.get('wrench_interaction', {})
    if wrench.get('state_source') != 'onboard' or wrench.get('shadow_mode') is not False:
        raise ValueError('active septic brake test requires active onboard wrench')
    if '_crazysim_model_verified' in wrench:
        raise ValueError('active septic brake test rejects simulation model markers')
    handoff = wrench.get('control_handoff', {})
    for name in (
            'coast_jerk_limited_attitude_enabled',
            'coast_jerk_limited_septic_smoothing_enabled',
            'coast_jerk_limited_free_stop_enabled'):
        if handoff.get(name) is not True:
            raise ValueError(f'active septic brake test requires {name}=true')

    target = mission.get('drones', {}).get(drone_id, {}).get('target')
    if not isinstance(target, (list, tuple)) or len(target) < 3:
        raise ValueError('active septic brake test requires drone target')
    yaw_deg = target[3] if len(target) > 3 else wrench.get('nominal_yaw_deg', 0.0)
    if not isinstance(yaw_deg, (int, float)) or not math.isfinite(yaw_deg):
        raise ValueError('active septic brake test requires finite nominal yaw')
    if sense_axis not in ('x', 'y') or sense_sign not in (-1, 1):
        raise ValueError('active septic brake test requires signed planar sensor axis')
    yaw = math.radians(yaw_deg)
    direction = (
        (sense_sign * math.cos(yaw), sense_sign * math.sin(yaw))
        if sense_axis == 'x' else
        (-sense_sign * math.sin(yaw), sense_sign * math.cos(yaw))
    )
    calibration_path = config.get('wrench_calibration_file', DEFAULT_CALIBRATION_PATH)
    try:
        _, calibration = apply_required_jerk_braking_calibration(
            wrench, drone_id, calibration_path,
            runtime_interaction_direction_xy=direction,
        )
    except ValueError as exc:
        raise ValueError(
            f'active septic brake test pre-arm calibration gate: {exc}'
        ) from exc
    if calibration is None or calibration.get('simulation_only'):
        raise ValueError('active septic brake test requires saved hardware calibration')
    return {
        'calibration_path': str(calibration_path),
        'runtime_direction_xy': direction,
        'planar_braking_fit_verified': True,
    }
