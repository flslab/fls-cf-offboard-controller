"""Detector-independent recording through controller setup, UART parsing and disk."""

from copy import deepcopy
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from Interaction.contact_validation_capture import PotentiometerSampleCapture
from Interaction.live_logger import LiveLogger
from Interaction.potentiometer_force_sensor import PotentiometerForceSensor
from Interaction.potentiometer_logging import (
    configure_potentiometer_recording,
    validate_potentiometer_recording,
)
from Interaction.tests.test_controller_logging_cleanup import methods


def arguments(**overrides):
    return SimpleNamespace(**{
        'interaction': True, 'sense': True, 'log': True, 'crazysim': True,
        'sense_port': '/unused', 'sense_baud': 115200,
        'sense_spring_constant': .2, 'sense_max_extension': 10.4,
        'sense_startup_timeout': 3., **overrides,
    })


def mission(detector='model', **options):
    return {'Interaction': {'config': {
        'behavior': 'level_coast', 'detection_method': detector,
        'record_potentiometer': True, **options,
    }}}


def replay_uart_start(sensor, startup_timeout_s):
    """Replace serial hardware only; keep the real reader/parser/capture path."""
    lines = iter([
        b'time_ms,raw,filtered,voltage,compression_mm\n',
        b'0,1000,990,4,99\n',
        b'10,880,990,4,99\n',
        b'20,549,990,4,99\n',  # Outside the calibrated range: invalid.
        b'30,900,990,4,99\n',
        b'40,1000,990,4,99\n',
    ])

    def readline():
        try:
            return next(lines)
        except StopIteration:
            sensor._stop_event.set()
            return b''

    sensor._serial = SimpleNamespace(readline=readline)
    sensor._read_loop()
    if sensor._reader_error is not None:
        raise sensor._reader_error


class PotentiometerRecordingTests(unittest.TestCase):
    def test_all_detectors_record_every_valid_uart_sample_with_calibration_metadata(self):
        setup = methods({'setup_force_sensor'}, logger=Mock())['setup_force_sensor']
        for detector in ('model', 'vel', 'potentiometer'):
            for shared_capture in (False, True):
                with self.subTest(detector=detector, shared_capture=shared_capture), TemporaryDirectory() as folder:
                    path = Path(folder) / 'flight.json'
                    writer = LiveLogger(path)
                    logs = SimpleNamespace(live_logger=writer)
                    if shared_capture:
                        logs.contact_validation_pot_callback = PotentiometerSampleCapture(writer)
                    config, args = mission(detector), arguments()
                    before = deepcopy((config, vars(args)))
                    controller = SimpleNamespace(args=args, mission=config, log_manager=logs,
                                                 rpi_power_monitor=None)
                    try:
                        with patch.object(PotentiometerForceSensor, 'start', replay_uart_start):
                            setup(controller)
                        if shared_capture:
                            self.assertIs(controller.force_sensor.sample_callback,
                                          logs.contact_validation_pot_callback)
                    finally:
                        writer.close()
                    records = json.loads(path.read_text())
                    self.assertEqual((config, vars(args)), before)
                    metadata = records[0]
                    self.assertEqual(metadata['type'], 'potentiometer_recording')
                    self.assertEqual(metadata['data']['primary_detector'], detector)
                    calibration = metadata['data']['compression_calibration']
                    self.assertEqual(calibration['method'], 'monotone cubic (PCHIP)')
                    self.assertEqual(calibration['raw_compression_mm_points'], [
                        [1000, 0.], [931, 1.6], [900, 4.5], [860, 6.8],
                        [841, 7.9], [731, 9.7], [550, 10.4]])
                    self.assertEqual(metadata['data']['force_model']['spring_constant_n_per_mm'], .2)
                    self.assertEqual([r['type'] for r in records[1:]], ['potentiometer_raw'] * 4)
                    samples = [r['data'] for r in records[1:]]
                    self.assertEqual([s['sample_sequence'] for s in samples], [0, 1, 2, 3])
                    self.assertEqual([s['arduino_time_ms'] for s in samples], [0, 10, 30, 40])
                    expected_mm = [0., 5.721140216711513, 4.5, 0.]
                    for sample, mm in zip(samples, expected_mm):
                        self.assertAlmostEqual(sample['compression_mm'], mm)
                        self.assertAlmostEqual(sample['force_n'], mm * .2)
                        self.assertGreater(sample['host_monotonic_time'], 0.)
                        self.assertEqual(sample['time'], sample['host_time'])
                        self.assertFalse(sample['command_authority'])
                    self.assertEqual(controller.force_sensor.latest().compression_mm, 0.)

    def test_disabled_adds_no_recording_and_preserves_existing_capture(self):
        for config in ({}, mission(record_potentiometer=False)):
            logs = SimpleNamespace(live_logger=Mock())
            self.assertIsNone(configure_potentiometer_recording(logs, config, arguments()))
            callback = PotentiometerSampleCapture(logs.live_logger)
            logs.contact_validation_pot_callback = callback
            self.assertIs(configure_potentiometer_recording(logs, config, arguments()), callback)
            logs.live_logger.write.assert_not_called()

    def test_incomplete_or_malformed_request_fails_before_opening_sensor(self):
        setup = methods({'setup_force_sensor'}, logger=Mock())['setup_force_sensor']
        cases = [(mission(record_potentiometer=value), arguments())
                 for value in ('true', 1, None, {})]
        cases += [(mission(), arguments(**option)) for option in (
            {'sense': False}, {'log': False}, {'droneless': True})]
        cases += [(mission(), arguments())]  # No flight logger available.
        with patch.object(PotentiometerForceSensor, 'start') as start:
            for config, args in cases:
                controller = SimpleNamespace(mission=config, args=args, log_manager=None)
                with self.subTest(config=config, args=args), self.assertRaisesRegex(ValueError, 'record_potentiometer'):
                    setup(controller)
            start.assert_not_called()

    def test_request_validated_even_when_logging_is_disabled(self):
        setup = methods({'setup_logging'}, logger=Mock())['setup_logging']
        with self.assertRaisesRegex(ValueError, 'record_potentiometer requires'):
            setup(SimpleNamespace(mission=mission(), args=arguments(log=False)))

    def test_other_flight_modes_do_not_inherit_recording_request(self):
        for mode in ('calibrate', 'braking_test', 'mpc', 'baseline', 'hover',
                     'active_septic_brake_calibration'):
            with self.subTest(mode=mode):
                self.assertFalse(validate_potentiometer_recording(
                    mission(), arguments(interaction=False, sense=False, **{mode: True})))


if __name__ == '__main__':
    unittest.main()
