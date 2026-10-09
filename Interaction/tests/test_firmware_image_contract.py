"""Validate the shipped compiled firmware against the current mission profile."""
import gzip
import hashlib
import json
from pathlib import Path
import unittest

from Interaction.firmware_image_contract import check


class FirmwareImageContractTests(unittest.TestCase):
    release = Path(__file__).parents[1]/'firmware_images/estimator_xy_20261009_v2'

    def test_shipped_image_has_current_scurve_and_hover_parameter_contract(self):
        mission = {'Interaction': {'config': {
            'behavior': 'level_coast', 'wrench_interaction_profile': 'level_coast',
            'level_coast': {'coast_command_mode': 'scurve', 'estimator_hover_xy': True}}}}
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


if __name__ == '__main__':
    unittest.main()
