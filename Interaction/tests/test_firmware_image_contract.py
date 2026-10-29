"""Validate the shipped compiled firmware against the current mission profile."""
import gzip
import hashlib
import json
from pathlib import Path
import unittest

from Interaction.firmware_image_contract import check
from Interaction.mission_profiles import resolve_mission_profiles


class FirmwareImageContractTests(unittest.TestCase):
    release = Path(__file__).parents[1]/'firmware_images/estimator_xy_20261009_v2'

    def test_shipped_image_preserves_legacy_scurve_and_hover_parameter_contract(self):
        mission = {'Interaction': {'config': {
            'behavior': 'level_coast', 'wrench_interaction_profile': 'level_coast',
            'level_coast': {'coast_command_mode': 'scurve', 'estimator_hover_xy': True}}}}
        # This frozen image predates the optional 40-degree command envelope.
        mission = resolve_mission_profiles(mission)
        config = mission['Interaction']['config']
        del config['wrench_interaction_profile']
        del config['wrench_interaction']['firmware_auto_brake']['analytic_profile']['max_tilt_deg']
        result = check(self.release/'bolt.elf.gz', mission)
        self.assertTrue(result['passed'], result['missing'])
        self.assertTrue({'hlCommander.pRelCompP', 'hlCommander.pRelFric',
                         'eskfXY.api'}.issubset(result['required']))

    def test_published_image_and_checked_elf_match_release_fingerprints(self):
        manifest = json.loads((self.release/'build-manifest.json').read_text())
        binary = (self.release/'bolt-distance-eskfxy-26100902.bin').read_bytes()
        compressed = (self.release/'bolt.elf.gz').read_bytes()
        self.assertEqual(hashlib.sha256(binary).hexdigest(), manifest['binary_sha256'])
        self.assertEqual(hashlib.sha256(compressed).hexdigest(), manifest['elf_gzip_sha256'])
        self.assertEqual(hashlib.sha256(gzip.decompress(compressed)).hexdigest(), manifest['elf_sha256'])
        header = Path(__file__).parents[1]/'native_estimator3/estimator_xy_calibration.h'
        self.assertEqual(hashlib.sha256(header.read_bytes()).hexdigest(),
            manifest['hardware_sources']['src/modules/interface/kalman_core/estimator_xy_calibration.h'])

    def test_old_image_is_not_compatible_with_higher_deceleration(self):
        mission = {'Interaction': {'config': {
            'behavior': 'level_coast', 'wrench_interaction_profile': 'level_coast',
            'level_coast': {'coast_command_mode': 'scurve'},
            'wrench_interaction': {'firmware_auto_brake': {'stop_deceleration_m_s2': 4.0}}}}}
        result = check(self.release/'bolt.elf.gz', mission)
        self.assertFalse(result['passed'])
        self.assertEqual(result['missing'], ['hlCommander.pRelAMax',
                                            'hlCommander.pRelTMax', 'hlCommander.pRelTilt'])

    def test_current_40_degree_profile_requires_new_image_even_at_default_deceleration(self):
        mission = {'Interaction': {'config': {
            'behavior': 'level_coast', 'wrench_interaction_profile': 'level_coast',
            'level_coast': {'coast_command_mode': 'scurve'}}}}
        result = check(self.release/'bolt.elf.gz', mission)
        self.assertFalse(result['passed'])
        self.assertIn('hlCommander.pRelTilt',result['missing'])

    def test_reset_disable_requires_both_compiled_switches(self):
        mission = {'Interaction': {'config': {
            'behavior': 'level_coast', 'wrench_interaction_profile': 'level_coast',
            'level_coast': {'coast_command_mode': 'scurve'},
            'wrench_interaction': {'firmware_auto_brake': {
                'reset_on_mode_change': False, 'reset_xy_on_scurve_transition': False}}}}}
        old = check(self.release/'bolt.elf.gz', mission)
        self.assertIn('pidCtl.resetMode', old['missing'])
        self.assertIn('hlCommander.pRelRstXY', old['missing'])
        candidate = self.release.parent/'scurve_tilt40_default_20261029/bolt.elf.gz'
        result = check(candidate, mission)
        self.assertTrue(result['passed'], result['missing'])


if __name__ == '__main__':
    unittest.main()
