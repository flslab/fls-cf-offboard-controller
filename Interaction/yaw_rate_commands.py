"""Explicit direct-yaw-rate CRTP packets; requires the paired PID firmware."""

import math
import struct

from cflib.crtp.crtpstack import CRTPPacket, CRTPPort

from Interaction.firmware_parameter_confirmation import confirm_firmware_mode_parameters

POSITION_YAW_RATE = 12
ZDISTANCE_YAW_RATE = 13


class DirectYawRateCommander:
    def __init__(self, cf):
        self.cf = cf

    def _send(self, packet_type, *values):
        values = tuple(float(v) for v in values)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('direct yaw-rate setpoints must be finite')
        if not getattr(self.cf, '_level_coast_yaw_rate_ready', False):
            raise RuntimeError('direct yaw-rate firmware was not confirmed before flight')
        packet = CRTPPacket()
        packet.port = CRTPPort.COMMANDER_GENERIC
        packet.channel = 0
        packet.data = struct.pack('<Bffff', packet_type, *values)
        self.cf.send_packet(packet)

    def send_position_yaw_rate_setpoint(self, x, y, z, yaw_rate):
        self._send(POSITION_YAW_RATE, x, y, z, yaw_rate)

    def send_zdistance_yaw_rate_setpoint(self, roll, pitch, yaw_rate, z):
        self._send(ZDISTANCE_YAW_RATE, roll, pitch, yaw_rate, z)


def prepare_yaw_rate_damping(cf, mission, controller_type, *, calibrating=False):
    """Fresh capability and selected-controller readback before arming."""
    cf._level_coast_yaw_rate_ready = False
    config = (mission or {}).get('Interaction', {}).get('config', {})
    if (calibrating or config.get('behavior') != 'level_coast'
            or not (config.get('level_coast') or {}).get('yaw_rate_damping', False)):
        return
    if controller_type != 'pid':
        raise ValueError('level_coast.yaw_rate_damping requires the PID controller')
    toc = getattr(getattr(cf.param, 'toc', None), 'toc', {})
    if 'version' not in toc.get('yawRate', {}):
        raise RuntimeError('yaw-rate damping requires updated firmware (yawRate.version=1)')
    confirm_firmware_mode_parameters(cf.param, expected={
        'yawRate.version': 1, 'stabilizer.controller': 1,
    })
    cf._level_coast_yaw_rate_ready = True
