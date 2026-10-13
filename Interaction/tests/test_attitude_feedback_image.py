"""Bind the feedback fix's shipped image to its checked ELF and source patch."""
import gzip
import hashlib
import json
from pathlib import Path
import unittest

from Interaction.firmware_image_contract import check


class AttitudeFeedbackImageTests(unittest.TestCase):
    release = Path(__file__).parents[1] / 'firmware_images/attitude_feedback_20261012'

    def test_compiled_image_supports_scurve_and_xy_mission(self):
        mission = {'Interaction': {'config': {
            'behavior': 'level_coast', 'wrench_interaction_profile': 'level_coast',
            'level_coast': {'coast_command_mode': 'scurve', 'estimator_hover_xy': True}}}}
        result = check(self.release / 'bolt.elf.gz', mission)
        self.assertTrue(result['passed'], result['missing'])

    def test_image_fingerprints_and_exact_patched_source(self):
        manifest = json.loads((self.release / 'build-manifest.json').read_text())
        patch = json.loads((self.release / 'patches/source.json').read_text())
        compressed = (self.release / 'bolt.elf.gz').read_bytes()
        for content, key in (
            ((self.release / manifest['binary_file']).read_bytes(), 'binary_sha256'),
            (compressed, 'elf_gzip_sha256'),
            (gzip.decompress(compressed), 'elf_sha256'),
            ((self.release / 'bolt.config').read_bytes(), 'hardware_config_sha256'),
        ):
            self.assertEqual(hashlib.sha256(content).hexdigest(), manifest[key])
        source = 'src/modules/src/estimator/estimator_kalman.c'
        self.assertEqual(patch['after_sha256'], manifest['hardware_sources'][source])
        self.assertEqual(patch['after_sha256'], manifest['firmware_sources'][source])
        self.assertIn(manifest['tag'].encode(), gzip.decompress(compressed))


if __name__ == '__main__':
    unittest.main()
