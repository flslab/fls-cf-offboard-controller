"""Preflight reset policy; old firmware remains usable with the default policy."""

RESET_PARAMETERS = {
    'reset_on_mode_change': 'pidCtl.resetMode',
    'reset_xy_on_scurve_transition': 'hlCommander.pRelRstXY',
}


def reset_parameters(config, toc=None):
    expected = {}
    for option, key in RESET_PARAMETERS.items():
        value = config.get(option, True)
        if type(value) is not bool:
            raise ValueError(f'firmware_auto_brake.{option} must be boolean')
        group, name = key.split('.')
        # Explicit disable requires firmware support. On supporting firmware,
        # omission restores default 1 rather than inheriting a previous flight.
        if toc is None or not value or name in toc.get(group, {}):
            expected[key] = int(value)
    return expected
