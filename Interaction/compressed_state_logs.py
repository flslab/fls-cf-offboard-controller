"""Decode the two existing-firmware CRTP blocks used by auto-brake.

These values are telemetry for Pi contact rendering and diagnosis. Firmware
braking continues to use its own full-precision estimator state.
"""

import numpy as np
from cflib.utils.encoding import decompress_quaternion

from Interaction.contact_attitude_observer import legacy_rpy_from_quaternion


# The two 100 Hz blocks are started separately, but a missing whole cycle
# must not be mistaken for a matched snapshot (10 ms apart).
MAX_PAIR_SKEW_S = 0.009


def _triplet(packet, names, scale):
    values = np.asarray([packet[name] for name in names], dtype=float)
    if values.shape != (3,) or not np.all(np.isfinite(values)):
        raise ValueError('compressed state triplet is missing or nonfinite')
    return scale * values


def decode_kinematic_packet(packet):
    """Return world XYZ position/velocity and legacy body RPY/rates in SI."""
    prefix = 'stateEstimateZ.'
    position = _triplet(packet, [prefix + axis for axis in 'xyz'], 0.001)
    velocity = _triplet(packet, [prefix + 'v' + axis for axis in 'xyz'],
                        0.001)
    rates = _triplet(packet, [prefix + 'rate' + axis for axis in
                              ('Roll', 'Pitch', 'Yaw')], 0.001)
    packed = packet[prefix + 'quat']
    if isinstance(packed, bool) or not isinstance(packed, (int, np.integer)):
        raise ValueError('compressed quaternion is not an integer')
    if not 0 <= int(packed) <= 0xFFFFFFFF:
        raise ValueError('compressed quaternion is out of range')
    x, y, z, w = decompress_quaternion(int(packed))
    attitude = legacy_rpy_from_quaternion((w, x, y, z))
    if not np.all(np.isfinite(attitude)):
        raise ValueError('compressed quaternion did not decode to finite RPY')
    return position, velocity, attitude, rates


def validate_actuator_packet(packet):
    """Require complete, finite motors/battery and compressed acceleration."""
    _triplet(packet, ['stateEstimateZ.a' + axis for axis in 'xyz'], 0.001)
    for index in range(1, 5):
        value = packet[f'motor.m{index}']
        if not isinstance(value, (int, float, np.number)) or not np.isfinite(value):
            raise ValueError('compressed actuator packet has invalid motor')
    battery = packet['pm.vbat']
    if not isinstance(battery, (int, float, np.number)) or not np.isfinite(battery):
        raise ValueError('compressed actuator packet has invalid battery')
