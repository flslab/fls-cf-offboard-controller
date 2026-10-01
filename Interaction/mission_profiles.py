"""Resolve named, repository-owned interaction profiles before flight setup."""

from copy import deepcopy
from pathlib import Path

import yaml


PROFILES = {'level_coast': Path(__file__).with_name('profiles') / 'level_coast.yaml'}


def _merge(base, overrides):
    result = deepcopy(base)
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def resolve_mission_profiles(mission):
    """Merge profile < mission overrides; saved XYZ calibration is applied later.

    The profile name remains in the resolved mission for logging. Inline-only
    missions retain their existing values. A missing/unknown profile fails at
    mission loading rather than silently changing to generic flight defaults.
    """
    resolved = deepcopy(mission)
    config = resolved.get('Interaction', {}).get('config', {})
    if 'wrench_interaction_profile' not in config:
        return resolved
    name = config['wrench_interaction_profile']
    if not isinstance(name, str) or name not in PROFILES:
        raise ValueError(f'Unknown wrench_interaction_profile: {name!r}')
    overrides = config.get('wrench_interaction', {})
    if not isinstance(overrides, dict):
        raise ValueError('wrench_interaction overrides must be a mapping')
    with PROFILES[name].open() as stream:
        profile = yaml.safe_load(stream)
    if not isinstance(profile, dict):
        raise ValueError(f'Interaction profile {name!r} must be a mapping')
    config['wrench_interaction'] = _merge(profile, overrides)
    return resolved
