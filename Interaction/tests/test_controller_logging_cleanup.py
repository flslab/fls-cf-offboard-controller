"""Logging diagnostics must not disable existing landing and cleanup behavior."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from Interaction.command_wrapper import CommandWrapper
from Interaction.live_logger import LiveLoggerError

ROOT = Path(__file__).resolve().parents[2]


def methods(names, **namespace):
    tree = ast.parse((ROOT/'controller.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Controller')
    selected = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    exec(compile(ast.Module(body=selected, type_ignores=[]), 'controller.py', 'exec'), namespace)
    return namespace


class LoggingCleanupTests(unittest.TestCase):
    def test_sticky_logger_failure_does_not_block_existing_landing(self):
        namespace = methods({'land'}, time=Mock(), logger=Mock(), CommandWrapper=CommandWrapper)
        failure = LiveLoggerError('disk failed')
        record = Mock(side_effect=failure)
        high, low = Mock(), Mock()
        live_logger = Mock()
        live_logger.stats_snapshot.return_value = {'error': 'disk failed'}
        logs = Mock(live_logger=live_logger)
        logs.get_latest_cf_log_data.side_effect = [0.1, 0.2, 1.0]
        controller = SimpleNamespace(
            args=SimpleNamespace(skip_landing=False, takeoff_altitude=1., init_yaw=0., vicon=True),
            hl_commander=CommandWrapper(high, record), ll_commander=CommandWrapper(low, record),
            cf=Mock(), log_manager=logs, voltage=7.4, init_coord=[0, 0, 0],
            flying=True, mission={'takeoff_speed': .5}, use_flowdeck=False,
            _send_landing_confirmation=Mock(),
        )
        original_logger = controller.hl_commander.log_function
        with self.assertLogs('Interaction.command_wrapper', level='ERROR'):
            namespace['land'](controller)
        low.send_notify_setpoint_stop.assert_called_once_with()
        high.go_to.assert_called_once()
        high.land.assert_called_once_with(.1, 2.)
        high.stop.assert_called_once_with()
        self.assertFalse(controller.flying)
        self.assertIs(controller.hl_commander.log_function, original_logger)
        controller._send_landing_confirmation.assert_called_once_with(7.4)
        with self.assertRaises(LiveLoggerError):
            controller.hl_commander.takeoff(1, 2)
        high.takeoff.assert_not_called()

    def test_close_failure_reaches_disconnect_and_is_still_reported(self):
        namespace = methods({'stop'}, time=SimpleNamespace(time=lambda: 10), logger=Mock())
        failure = LiveLoggerError('final flush failed')
        logs = Mock()
        logs.stop.side_effect = failure
        controller = SimpleNamespace(
            mission_start_time=0, servo=None, land=Mock(), bat_logger=None, mocap=None,
            force_sensor=None, rpi_power_monitor=None, log_manager=logs,
            args=SimpleNamespace(log_dir='/unused', tag='test'), animation_start_times=[],
            animation_stop_times=[], viewpoint_offsets=[], reference_offsets=[],
            tracker_process=None, blinker_process=None, smooth_controller=Mock(),
            led=Mock(), disconnect=Mock(),
        )
        with self.assertRaises(LiveLoggerError) as caught:
            namespace['stop'](controller)
        self.assertIs(caught.exception, failure)
        controller.land.assert_called_once()
        controller.smooth_controller.stop.assert_called_once()
        controller.led.stop.assert_called_once()
        controller.disconnect.assert_called_once()

    def test_setup_registers_group_before_start_for_strict_loggers(self):
        mocap_factory, clock = Mock(), Mock()
        logs = SimpleNamespace(groups={})
        logs.add_log_group = lambda name: logs.groups.setdefault(name, [])
        logs.add_log_entry = lambda group, data: logs.groups[group].append(data)
        namespace = methods({'setup_motion_capture'}, time=clock, logger=Mock(), Mocap=mocap_factory)
        controller = SimpleNamespace(
            args=SimpleNamespace(vicon=True, save_vicon=False, vicon_full_pose=False,
                                 vicon_mode='pointcloud', init_pos=[0, 0, 0], drone_id='test'),
            log_manager=logs, mission={}, _log_mocap=Mock(), _send_position=Mock(),
            _log_mocap_timing=lambda data: logs.add_log_entry('mocap_timing', data),
        )
        mocap_factory.return_value.start.side_effect = lambda: controller._log_mocap_timing({'frame_id': 0})
        namespace['setup_motion_capture'](controller)
        self.assertEqual(logs.groups['mocap_timing'], [{'frame_id': 0}])
        self.assertIs(mocap_factory.call_args.kwargs['timing_callback'], controller._log_mocap_timing)
        controller.log_manager = None
        mocap_factory.return_value.start.side_effect = None
        namespace['setup_motion_capture'](controller)
        self.assertIsNone(mocap_factory.call_args.kwargs['timing_callback'])


if __name__ == '__main__':
    unittest.main()
