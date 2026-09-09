"""Fixed, data-only repeat protocol; never installs a flight calibration."""

from copy import deepcopy
import hashlib
from pathlib import Path

from Interaction.braking_response_calibration import PlanarBrakingCalibration
from Interaction.calibration_trial_readiness import CalibrationTrialReadinessGate


def resolve_repeat_test_selection(direction=None, repetitions=None):
    """Default to six trials; restrict supplemental collection to the same axis."""
    direction = 'both' if direction is None else direction
    repetitions = 3 if repetitions is None else repetitions
    if direction not in ('both', 'positive-y', 'negative-y'):
        raise ValueError('--braking-test-direction must be both, positive-y or negative-y')
    if (isinstance(repetitions, bool) or not isinstance(repetitions, int)
            or not 1 <= repetitions <= 3):
        raise ValueError('--braking-test-repetitions must be an integer from 1 to 3')
    return direction, repetitions


def repeat_test_config(config, *, direction=None, repetitions=None):
    """Override the maneuver, not the mission's safety/readiness envelope."""
    direction, repetitions = resolve_repeat_test_selection(direction, repetitions)
    directions = {
        'both': [[0.0, -1.0], [0.0, 1.0]],
        'positive-y': [[0.0, 1.0]],
        'negative-y': [[0.0, -1.0]],
    }[direction]
    result = deepcopy(config)
    result['shadow_mode'] = True
    result['startup_bias_calibration_enabled'] = True
    result.setdefault('calibration_excitation', {})['enabled'] = False
    result.setdefault('guided_touch_test', {})['enabled'] = False
    result.pop('position_capture_calibration', None)
    braking = result.setdefault('planar_braking_calibration', {})
    braking.update({
        'enabled': True,
        'tilt_levels_deg': [20.0],
        'accelerate_durations_s': [0.24],
        'repetitions_per_duration': repetitions,
        'directions_xy': directions,
        'start_delay_s': 1.0,
        'level_before_acceleration_s': 0.20,
        'level_before_brake_s': 0.20,
        'level_after_brake_s': 0.65,
        'recovery_s': 2.0,
    })
    # Fail before maneuvers for malformed inherited limits.
    PlanarBrakingCalibration(braking, start_after_s=0.0, require_opposed_directions=False)
    CalibrationTrialReadinessGate(braking)
    return result


def validate_repeat_test_options(args):
    """Validate dedicated CLI mode without importing hardware dependencies."""
    direction = getattr(args, 'braking_test_direction', None)
    repetitions = getattr(args, 'braking_test_repetitions', None)
    if not getattr(args, 'braking_test', False):
        if direction is not None or repetitions is not None:
            raise ValueError('--braking-test-direction and --braking-test-repetitions require --braking-test')
        return
    resolve_repeat_test_selection(direction, repetitions)
    conflicts = (
        'interaction', 'calibrate', 'sense', 'illumination',
        'intractable_illumination', 'autotune', 'simple_takeoff',
        'rotation_test', 'xy_tune', 'z_tune', 'trajectory',
    )
    for name in conflicts:
        if getattr(args, name, False):
            raise ValueError('--braking-test cannot be combined with --' + name.replace('_', '-'))
    if not getattr(args, 'log', False):
        raise ValueError('--braking-test requires --log to record every trial')
    if getattr(args, 'smooth_controller_rate', 0) < 50:
        raise ValueError('--braking-test requires --smooth-controller-rate 50 or higher')
    if getattr(args, 'cf_log_period', 20) not in (10, 20):
        raise ValueError('--braking-test requires --cf-log-period 10 or 20 (milliseconds)')


def calibration_reference(path):
    """Record which existing artifact was present; no parsing or writes."""
    path = Path(path)
    try:
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
    except FileNotFoundError:
        digest = None
    return {'path': str(path), 'sha256': digest}


def repeat_test_protocol(plan, config):
    gate = CalibrationTrialReadinessGate(config)
    return {
        **plan.timing_protocol(),
        'directions_xy': plan.directions.tolist(),
        'trial_directions_xy': plan.trial_directions.tolist(),
        'direction_selection': (
            'both' if len(plan.directions) == 2
            else 'positive-y' if plan.directions[0, 1] > 0 else 'negative-y'
        ),
        'repetitions_per_direction': plan.repetitions_per_duration,
        'tilt_deg': plan.tilt_deg,
        'tilt_levels_deg': plan.tilt_levels_deg.tolist(),
        'accelerate_s': plan.accelerate_s,
        'brake_s': plan.brake_s,
        'level_before_acceleration_s': plan.level_before_acceleration_s,
        'level_before_brake_s': plan.level_before_brake_s,
        'level_after_brake_s': plan.level_after_brake_s,
        'recovery_s': plan.recovery_s,
        'max_xy_speed_m_s': plan.max_xy_speed_m_s,
        'max_displacement_m': plan.max_displacement_m,
        **{name: getattr(gate, name) for name in (
            'trial_start_max_xy_speed_m_s', 'trial_start_max_tilt_deg',
            'trial_start_max_position_error_m', 'trial_start_dwell_s',
            'trial_start_timeout_s', 'trial_start_max_sample_gap_s',
        )},
    }


def repeat_test_result(plan, config, samples, reference):
    """Completion is data completeness, explicitly not model quality approval."""
    expected = set(range(len(plan.trial_directions)))
    if set(sample['segment_id'] for sample in samples) != expected:
        raise ValueError('braking repeat test is missing trial samples; calibration preserved')
    counts = []
    for segment in sorted(expected):
        rows = [sample for sample in samples if sample['segment_id'] == segment]
        if {row['phase'] for row in rows} != plan.ATTITUDE_PHASES:
            raise ValueError(f'braking repeat trial {segment} has incomplete phases; calibration preserved')
        counts.append(len(rows))
    return {
        'protocol': repeat_test_protocol(plan, config),
        'maneuver_count': len(expected),
        'sample_count': len(samples),
        'trial_sample_counts': counts,
        'calibration_reference': reference,
        'previous_calibration_preserved': True,
        'offline_only': True,
        'model_fitted': False,
        'instruction': 'Validate independently against the previous frozen model; do not deploy a fit from this event.',
    }
