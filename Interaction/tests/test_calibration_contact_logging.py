"""Ordinary calibration captures data, never enables a detector/controller."""

import ast
from copy import deepcopy
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from Interaction import config
from Interaction.calibration_contact_logging import (
    FORCE_IMU, calibration_log_vars, configure_calibration_capture,
    is_plain_xyz_calibration,
)
from Interaction.curve_logging import curve_state_log_vars
from Interaction.live_logger import LiveLogger
from Interaction.tests import test_log_packet_timestamps as packet_fixture
from Interaction.tests.test_interaction_log_period import FakeLogConfig

ROOT = Path(__file__).resolve().parents[2]


def fake_cf(selected):
    toc = {}
    for group in selected.values():
        for field in group:
            if field != 'log_period_ms':
                prefix, name = field.split('.', 1)
                toc.setdefault(prefix, {})[name] = object()
    return SimpleNamespace(log=SimpleNamespace(toc=SimpleNamespace(toc=toc)),
                           param=SimpleNamespace(toc=SimpleNamespace(toc={})))


class CalibrationCaptureTests(unittest.TestCase):
    def test_only_plain_calibration_enables_capture(self):
        self.assertTrue(is_plain_xyz_calibration(SimpleNamespace(calibrate=True)))
        self.assertFalse(is_plain_xyz_calibration(SimpleNamespace()))
        self.assertFalse(is_plain_xyz_calibration(SimpleNamespace(interaction=True)))
        for mode in ('interaction', 'braking_test', 'targeted_braking_calibration',
                     'adaptive_braking_calibration', 'planar_braking_calibration',
                     'mpc', 'ground_test', 'droneless'):
            with self.subTest(mode=mode):
                self.assertFalse(is_plain_xyz_calibration(
                    SimpleNamespace(calibrate=True, **{mode: True})))

    def test_raw_block_is_24_bytes_body_specific_force_and_gyro(self):
        self.assertEqual(FORCE_IMU['log_period_ms'], 10)
        self.assertEqual(len(FORCE_IMU) - 1, 6)
        self.assertEqual({k for k in FORCE_IMU if k != 'log_period_ms'},
                         {f'{s}.{a}' for s in ('acc', 'gyro') for a in 'xyz'})
        for axis in 'xyz':
            self.assertEqual(FORCE_IMU[f'acc.{axis}']['unit'], 'g')
            self.assertEqual(FORCE_IMU[f'gyro.{axis}']['unit'], 'deg/s')
        self.assertTrue(all(v['type'] == 'float' for k, v in FORCE_IMU.items()
                            if k != 'log_period_ms'))

    def test_legacy_groups_preserved_without_duplicate_attitude_targets(self):
        before = deepcopy(config.LOG_VARS)
        result = calibration_log_vars(config.LOG_VARS)
        self.assertEqual(config.LOG_VARS, before)
        self.assertEqual(set(result) - set(before), {'FORCE_IMU'})
        self.assertNotIn('ATT_DES', result)
        for group in before:
            self.assertEqual(result[group]['log_period_ms'], 10)
        result['VEL_ORI']['stateEstimate.vx']['type'] = 'FP16'
        self.assertEqual(config.LOG_VARS, before)

    def test_compressed_groups_get_only_missing_attitude_and_imu(self):
        mission = {'Interaction': {'config': {'wrench_interaction': {
            'firmware_auto_brake': {'enabled': True}}}}}
        before = config.log_vars_for_mission(mission)
        result = calibration_log_vars(before)
        self.assertEqual(set(result) - set(before), {'ATT_DES', 'FORCE_IMU'})
        self.assertEqual(result['FIRMWARE_BRAKE']['log_period_ms'], 100)
        self.assertEqual(result['ATT_DES']['log_period_ms'], 10)
        curved = curve_state_log_vars(before, events_enabled=False)
        self.assertEqual(set(calibration_log_vars(curved)) - set(curved), {'FORCE_IMU'})

    def setup_capture(self, tmp, selected=None, *, crazysim=False):
        path = Path(tmp) / 'calibration.json'
        mission = {'Interaction': {'config': {'wrench_calibration_file': str(path),
            'wrench_interaction': {'detection': {'translation': {
                'component_thresholds': [.08, .08, .12]}}}}}}
        args = SimpleNamespace(cf_log_period=20, drone_id='lb11', crazysim=crazysim)
        selected = config.LOG_VARS if selected is None else selected
        cf = fake_cf(calibration_log_vars(selected))
        logger = Mock()
        logger.capture_packet_timing = False
        return path, mission, args, cf, logger, selected

    def test_manifest_freezes_previous_model_and_does_not_write_calibration(self):
        with TemporaryDirectory() as tmp:
            path, mission, args, cf, logger, selected = self.setup_capture(tmp)
            contents = json.dumps({'schema_version': 1, 'drones': {
                'lb11': {'model_delay_s': [.1, .2, .3]},
                'other': {'model_delay_s': [1, 2, 3]}}})
            path.write_text(contents)
            before = deepcopy(mission)
            result = configure_calibration_capture(logger, cf, selected, mission, args)
            record = logger.live_logger.write.call_args.args[0]
            self.assertEqual(record['type'], 'calibration_contact_capture')
            manifest = record['data']
            self.assertTrue(logger.capture_packet_timing)
            self.assertTrue(manifest['capture_only'])
            self.assertFalse(manifest['imu_atomic_sensor_snapshot'])
            self.assertEqual(manifest['groups']['FORCE_IMU']['payload_bytes'], 24)
            self.assertEqual(manifest['calibration_before_flight']['status'], 'saved')
            self.assertEqual(manifest['calibration_before_flight']['entry'],
                             {'model_delay_s': [.1, .2, .3]})
            self.assertEqual(len(manifest['calibration_before_flight']['sha256']), 64)
            self.assertEqual(path.read_text(), contents)
            self.assertEqual(mission, before)
            self.assertEqual(set(result) - set(selected), {'FORCE_IMU'})
            mission.clear()
            self.assertEqual(manifest['mission_before_calibration'], before)
            json.dumps(manifest)  # ready for the real asynchronous writer

    def test_missing_previous_calibration_is_explicit_not_fabricated(self):
        with TemporaryDirectory() as tmp:
            path, mission, args, cf, logger, selected = self.setup_capture(tmp)
            configure_calibration_capture(logger, cf, selected, mission, args)
            snapshot = logger.live_logger.write.call_args.args[0]['data']['calibration_before_flight']
            self.assertEqual(snapshot['status'], 'missing')
            self.assertIsNone(snapshot['entry'])
            self.assertFalse(path.exists())

    def test_missing_toc_rejected_before_starting_any_log_blocks(self):
        with TemporaryDirectory() as tmp:
            _, mission, args, cf, logger, selected = self.setup_capture(tmp)
            del cf.log.toc.toc['acc']['y']
            with self.assertRaisesRegex(RuntimeError, r'missing firmware log variables: acc.y'):
                configure_calibration_capture(logger, cf, selected, mission, args)
            logger.live_logger.write.assert_not_called()
            self.assertFalse(logger.capture_packet_timing)

    def test_payload_and_block_budget(self):
        oversized = {'TOO_BIG': {f'acc.a{i}': {'type': 'float'} for i in range(7)}}
        too_many = {f'G{i}': {'acc.x': {'type': 'float'}} for i in range(14)}
        too_many_fields = {f'G{i}': {f'acc.a{j}': {'type': 'uint8_t'}
                                   for j in range(10)} for i in range(13)}
        for selected, message in ((oversized, '26 bytes'), (too_many, '15-block'),
                                  (too_many_fields, '127-variable')):
            with TemporaryDirectory() as tmp:
                _, mission, args, cf, logger, selected = self.setup_capture(tmp, selected)
                with self.assertRaisesRegex(ValueError, message):
                    configure_calibration_capture(logger, cf, selected, mission, args)
                logger.live_logger.write.assert_not_called()

    def test_real_writer_saves_imu_and_timestamps_without_a_live_listener(self):
        fixture = packet_fixture.LogPacketTimestampTests()
        logger = fixture.make_logger()
        logger.capture_packet_timing = True
        logger.cf_var_logger = []
        with TemporaryDirectory() as tmp:
            path = Path(tmp) / 'flight.json'
            logger.live_logger = LiveLogger(str(path))
            with patch('Interaction.log_manager.time.time', side_effect=[100., 100.01]), \
                    patch('Interaction.log_manager.time.monotonic', side_effect=[50., 50.01]):
                for timestamp, group, data in (
                        (0xFFFFFE, 'FORCE_IMU', {'acc.x': .02, 'gyro.x': 7.}),
                        (8, 'VEL_ORI', {'stateEstimate.vy': .2})):
                    logger._cf_log_group_callback(timestamp, data, SimpleNamespace(name=group))
            logger.stop()
            records = json.loads(path.read_text())
            self.assertEqual([r['data']['cf_packet_sequence'] for r in records], [0, 1])
            self.assertEqual([r['data']['cf_timestamp_ms'] for r in records], [0xFFFFFE, 8])
            self.assertEqual([r['data']['host_receive_monotonic_s'] for r in records], [50., 50.01])
            self.assertEqual(records[0]['data']['acc.x'], .02)
            self.assertEqual(logger._cf_log_packet_listeners, [])
            self.assertNotIn('cf_timestamp_ms', logger.cf_log_group_packets['FORCE_IMU'][0])

    def test_capture_and_existing_curve_listener_share_one_sequence(self):
        logger = packet_fixture.LogPacketTimestampTests().make_logger()
        logger.capture_packet_timing = True
        received = []
        logger.add_cf_packet_listener(received.append)
        for tick in (10, 20):
            logger._cf_log_group_callback(tick, {'acc.x': .1}, SimpleNamespace(name='FORCE_IMU'))
        self.assertEqual([p.sequence for p in received], [0, 1])
        saved = [c.args[0]['data'] for c in logger.live_logger.write.call_args_list]
        self.assertEqual([p['cf_packet_sequence'] for p in saved], [0, 1])
        self.assertEqual([p['host_receive_monotonic_s'] for p in saved],
                         [p.host_receive_monotonic_s for p in received])

    def test_capture_period_uses_existing_bolt_and_sitl_encoding(self):
        for crazysim, expected_period in ((False, 100), (True, 10)):
            logger = packet_fixture.LogPacketTimestampTests().make_logger()
            logger.args = SimpleNamespace(crazysim=crazysim)
            FakeLogConfig.instances = []
            with patch('Interaction.log_manager.LogConfig', FakeLogConfig):
                logger.init_cf_logger(Mock(), {'FORCE_IMU': FORCE_IMU}, 20)
            self.assertEqual(FakeLogConfig.instances[0].period_in_ms, expected_period)

    def test_controller_wiring_is_scoped_and_after_curve_selection(self):
        tree = ast.parse((ROOT / 'controller.py').read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Controller')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'setup_logging')
        namespace = {'logger': Mock()}
        exec(compile(ast.Module(body=[method], type_ignores=[]), 'controller.py', 'exec'), namespace)
        for calibrate in (False, True):
            for curve_enabled in (False, True):
                with self.subTest(calibrate=calibrate, curve=curve_enabled):
                    logs = Mock()
                    selected = config.LOG_VARS
                    after_curve = curve_state_log_vars(selected, events_enabled=False)
                    args = SimpleNamespace(log=True, illumination=False, droneless=False,
                        calibrate=calibrate, crazysim=False, cf_log_period=20, log_dir='/unused', tag='test')
                    mission = {'Interaction': {'config': {'wrench_interaction': {
                        'firmware_auto_brake': {
                            'curve_log': {'enabled': curve_enabled}}}}}}
                    controller = SimpleNamespace(args=args, mission=mission, cfg=config,
                        cf=fake_cf({}), _is_interaction_application=lambda: True,
                        _uses_onboard_wrench_state=lambda: True,
                        _uses_vicon_velocity_for_free_stop=lambda: False,
                        firmware_auto_brake_enabled=False)
                    # Event logging on a normal interaction has its own preflight;
                    # only calibration suppresses events while retaining state logs.
                    if not calibrate and curve_enabled:
                        continue
                    with patch('Interaction.log_manager.InteractionLogger', return_value=logs), \
                            patch('Interaction.calibration_contact_logging.configure_calibration_capture',
                                  return_value={'CAPTURE': {}}) as capture, \
                            patch('Interaction.curve_logging.CurveRecorder'):
                        namespace['setup_logging'](controller)
                    if calibrate:
                        capture.assert_called_once_with(logs, controller.cf,
                            after_curve if curve_enabled else selected, mission, args)
                        self.assertEqual(logs.init_cf_logger.call_args.args[1], {'CAPTURE': {}})
                    else:
                        capture.assert_not_called()
                        self.assertEqual(logs.init_cf_logger.call_args.args[1], selected)


if __name__ == '__main__':
    unittest.main()
