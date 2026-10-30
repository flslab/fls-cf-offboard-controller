"""Manifest voltage selection and existing battery landing behavior; no hardware."""
from threading import Event
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from controller import Controller


class ManifestBatteryThresholdTests(unittest.TestCase):
    def subject(self, manifest=None, *, simulation=False):
        return SimpleNamespace(
            manifest=manifest, cfg=SimpleNamespace(MIN_LIHV_VOLT=3.5),
            args=SimpleNamespace(crazysim=simulation),
            battery_critical_consecutive_samples=2, low_voltage_sample_count=0,
            battery_critical=Event(), log_manager=Mock())

    def test_whole_pack_manifest_value_overrides_cell_default(self):
        subject = self.subject({'mission': {'battery_threshold': 5}})
        Controller.configure_battery_threshold(subject)
        self.assertEqual(subject.min_voltage, 5.0)
        self.assertEqual(subject.battery_threshold_source,
                         'manifest.mission.battery_threshold')

    def test_missing_setting_retains_seven_volt_default(self):
        for manifest in (None, {}, {'mission': {}}, {'mission': {'delta_t': 0}}):
            with self.subTest(manifest=manifest):
                subject = self.subject(manifest)
                Controller.configure_battery_threshold(subject)
                self.assertEqual(subject.min_voltage, 7.0)

    def test_invalid_setting_fails_instead_of_silently_falling_back(self):
        for value in (None, True, False, '5', 0, -1, float('nan'), float('inf')):
            with self.subTest(value=value):
                subject = self.subject({'mission': {'battery_threshold': value}})
                with self.assertRaisesRegex(ValueError, 'battery_threshold'):
                    Controller.configure_battery_threshold(subject)

    def test_simulation_retains_disabled_voltage_check(self):
        subject = self.subject({'mission': {'battery_threshold': 5}}, simulation=True)
        Controller.configure_battery_threshold(subject)
        self.assertEqual(subject.min_voltage, 0.0)

    def test_two_consecutive_low_samples_and_recovery_are_preserved(self):
        subject = self.subject({'mission': {'battery_threshold': 5}})
        Controller.configure_battery_threshold(subject)
        for voltage in (5.0, 4.9, 5.1, 4.8):
            Controller._watch_battery(subject, 0, {'pm.vbat': voltage}, None)
            self.assertFalse(subject.battery_critical.is_set())
        Controller._watch_battery(subject, 0, {'pm.vbat': 4.7}, None)
        self.assertTrue(subject.battery_critical.is_set())
        event = subject.log_manager.add_log_entry.call_args.args[1]
        self.assertEqual(event['threshold_v'], 5.0)
        self.assertEqual(event['consecutive_low_voltage_samples'], 2)

    def test_start_applies_manifest_before_watchers_or_arm(self):
        class StopSetup(Exception):
            pass

        order = []
        subject = SimpleNamespace(
            check_deck=lambda: order.append('deck'),
            load_manifest=lambda: order.append('manifest'),
            configure_battery_threshold=lambda: order.append('threshold'),
            setup_sockets=Mock(side_effect=StopSetup), arm=Mock(), takeoff=Mock())
        with self.assertRaises(StopSetup):
            Controller.start(subject)
        self.assertEqual(order, ['deck', 'manifest', 'threshold'])
        subject.arm.assert_not_called()
        subject.takeoff.assert_not_called()


if __name__ == '__main__':
    unittest.main()
