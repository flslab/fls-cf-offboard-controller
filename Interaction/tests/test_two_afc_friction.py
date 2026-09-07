import unittest

from Interaction.interactions import (
    PairedFrictionRandomizer,
    VirtualObjectPlanarMotion,
)


class StubBits:
    def __init__(self, bits):
        self.bits = iter(bits)

    def getrandbits(self, _count):
        return next(self.bits)


class PairedFrictionRandomizerTests(unittest.TestCase):
    def test_each_pair_contains_one_high_and_one_low_in_randomized_order(self):
        schedule = PairedFrictionRandomizer(
            {'enabled': True, 'high_mu': 0.10, 'low_mu': 0.01},
            rng=StubBits([1, 0, 1]),
        )

        rows = [schedule.begin_interaction() for _ in range(6)]

        self.assertEqual(
            [row['condition'] for row in rows],
            ['high', 'low', 'low', 'high', 'high', 'low'],
        )
        for pair_start in range(0, len(rows), 2):
            self.assertEqual(
                {rows[pair_start]['mu'], rows[pair_start + 1]['mu']},
                {0.10, 0.01},
            )
            self.assertEqual(
                rows[pair_start]['pair_number'],
                rows[pair_start + 1]['pair_number'],
            )

    def test_disabled_schedule_does_not_record_interactions(self):
        schedule = PairedFrictionRandomizer({'enabled': False})

        self.assertIsNone(schedule.begin_interaction())
        self.assertEqual(schedule.summary()['actual_sequence'], [])

    def test_invalid_coefficients_and_seed_are_rejected(self):
        invalid_configs = (
            {'enabled': True, 'high_mu': 0.01, 'low_mu': 0.01},
            {'enabled': True, 'high_mu': 0.01, 'low_mu': 0.10},
            {'enabled': True, 'high_mu': 0.10, 'low_mu': -0.01},
            {'enabled': True, 'random_seed': 1.5},
        )
        for config in invalid_configs:
            with self.subTest(config=config), self.assertRaises(ValueError):
                PairedFrictionRandomizer(config)

    def test_selected_mu_updates_both_virtual_friction_terms(self):
        motion = VirtualObjectPlanarMotion(
            mass=0.17,
            max_velocity_m_s=0.6,
            max_offset_xy=[1.0, 1.0],
            kinetic_friction_coefficient=0.10,
            static_friction_coefficient=0.10,
        )

        motion.set_friction_coefficients(0.01, 0.01)

        self.assertEqual(
            motion.resistance_config['kinetic_friction_coefficient'], 0.01
        )
        self.assertEqual(
            motion.resistance_config['static_friction_coefficient'], 0.01
        )


if __name__ == '__main__':
    unittest.main()
