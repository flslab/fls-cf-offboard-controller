"""Bounded, fresh parameter confirmation for opt-in pre-arm checks only."""

import math
import threading
import time


def _matches(observed, expected):
    """Floats are compared with the firmware's own float32 write tolerance."""
    if observed is None:
        return False
    if isinstance(expected, float):
        return abs(observed - expected) < 1e-3
    return observed == expected


def confirm_firmware_mode_parameters(param, *, timeout_s=5.0, expected=None):
    """Read back both Pi-planning switches, without trusting cflib's cache.

    set_value() queues asynchronous writes; get_value() only reads a cache.
    Register callbacks first, then queue explicit reads behind the setup
    writes. No parameter writes, sleep-based guesses, or flight commands.
    """
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError('parameter confirmation timeout must be finite and positive')
    expected = (dict(expected) if expected is not None else
                {'hlCommander.pRelJoint': 1, 'hlCommander.pRelHost': 1})
    if not expected or any(not isinstance(key, str) or key.count('.') != 1
                           or isinstance(value, bool)
                           or not isinstance(value, (int, float))
                           or not math.isfinite(value)
                           for key, value in expected.items()):
        raise ValueError('expected parameters must map group.name to numbers')
    observed = {}
    condition = threading.Condition()
    registered = []
    deadline = time.monotonic() + timeout_s

    def updated(name, value):
        if name not in expected:
            return
        try:
            parsed = (float(value) if isinstance(expected[name], float)
                      else int(value))
        except (ValueError, TypeError, OverflowError):
            parsed = None
        with condition:
            observed[name] = parsed
            condition.notify_all()

    try:
        for full_name in expected:
            group, name = full_name.split('.', 1)
            param.add_update_callback(group=group, name=name, cb=updated)
            registered.append((group, name))
        for full_name in expected:
            param.request_param_update(full_name)
        with condition:
            confirmed = condition.wait_for(
                lambda: all(_matches(observed.get(key), value)
                            for key, value in expected.items()),
                timeout=max(0., deadline-time.monotonic()))
            if not confirmed:
                detail = ', '.join(
                    f'{key}={observed.get(key, "no fresh reply")} (expected {value})'
                    for key, value in expected.items()
                    if not _matches(observed.get(key), value))
                raise RuntimeError('Pi event planner firmware mode not confirmed '
                                   f'before arm after {timeout_s:.1f}s: {detail}')
            return dict(observed)
    finally:
        for group, name in registered:
            param.remove_update_callback(group=group, name=name, cb=updated)
