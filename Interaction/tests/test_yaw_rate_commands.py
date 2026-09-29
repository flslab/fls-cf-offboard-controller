import struct
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from cflib.crtp.crtpstack import CRTPPort
from Interaction.command_wrapper import CommandWrapper
from Interaction.yaw_rate_commands import prepare_yaw_rate_damping


class YawRateCommandTests(unittest.TestCase):
    def make_cf(self):
        return SimpleNamespace(_level_coast_yaw_rate_ready=True, send_packet=Mock(),
            param=SimpleNamespace(toc=SimpleNamespace(toc={'yawRate': {'version': object()}})))

    def mission(self):
        return {'Interaction': {'config': {'behavior': 'level_coast',
                'level_coast': {'yaw_rate_damping': True}}}}

    def test_packets_preserve_position_offset_and_level_attitude_with_zero_yaw_rate(self):
        cf = self.make_cf()
        log = Mock()
        wrapper = CommandWrapper(SimpleNamespace(_cf=cf), log, offset=[1., 2., 3.])
        wrapper.send_position_yaw_rate_setpoint(.25, -.5, 1., 0.)
        packet = cf.send_packet.call_args.args[0]
        self.assertEqual((packet.port, packet.channel), (CRTPPort.COMMANDER_GENERIC, 0))
        self.assertEqual(struct.unpack('<Bffff', packet.data), (12, 1.25, 1.5, 4., 0.))
        wrapper.send_zdistance_yaw_rate_setpoint(0., 0., 0., 1.)
        self.assertEqual(struct.unpack('<Bffff', cf.send_packet.call_args.args[0].data),
                         (13, 0., 0., 0., 1.))
        self.assertEqual(log.call_count, 2)

    def test_no_packet_on_unconfirmed_firmware_nonfinite_input_log_failure_or_dry_run(self):
        for case in ('unconfirmed', 'nonfinite', 'log_failure', 'dry_run'):
            with self.subTest(case=case):
                cf = self.make_cf(); log = Mock()
                if case == 'unconfirmed': cf._level_coast_yaw_rate_ready = False
                if case == 'log_failure': log.side_effect = RuntimeError('log failure')
                wrapper = CommandWrapper(SimpleNamespace(_cf=cf), log, execute=case != 'dry_run')
                args = [0., 0., 1., float('nan') if case == 'nonfinite' else 0.]
                if case == 'dry_run':
                    wrapper.send_position_yaw_rate_setpoint(*args)
                else:
                    with self.assertRaises((ValueError, RuntimeError)):
                        wrapper.send_position_yaw_rate_setpoint(*args)
                cf.send_packet.assert_not_called()

    def test_preflight_requires_fresh_capability_and_actual_pid_readback(self):
        cf = self.make_cf()
        with patch('Interaction.yaw_rate_commands.confirm_firmware_mode_parameters') as confirm:
            prepare_yaw_rate_damping(cf, self.mission(), 'pid')
            confirm.assert_called_once_with(cf.param, expected={
                'yawRate.version': 1, 'stabilizer.controller': 1})
            self.assertTrue(cf._level_coast_yaw_rate_ready)
            confirm.side_effect = RuntimeError('unsupported version or controller')
            with self.assertRaises(RuntimeError):
                prepare_yaw_rate_damping(cf, self.mission(), 'pid')
            self.assertFalse(cf._level_coast_yaw_rate_ready)

    def test_old_firmware_and_other_controller_rejected_but_disabled_mode_is_compatible(self):
        cf = self.make_cf()
        with self.assertRaises(ValueError):
            prepare_yaw_rate_damping(cf, self.mission(), 'mellinger')
        cf.param.toc.toc = {}
        with self.assertRaises(RuntimeError):
            prepare_yaw_rate_damping(cf, self.mission(), 'pid')
        prepare_yaw_rate_damping(cf, {}, 'pid')
        self.assertFalse(cf._level_coast_yaw_rate_ready)
