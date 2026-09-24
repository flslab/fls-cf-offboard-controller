import unittest
from Interaction.firmware_analytic_profile import profile_parameters


class AnalyticProfileTests(unittest.TestCase):
    def config(self, **changes):
        return dict(shape='velocity_scurve', execution='velocity', tail_s=.35,
                    single_s=.35, handoff='predicted_bumpless', feedback='unified_vicon15', **changes)

    def test_none_preserves_legacy(self):
        self.assertEqual(profile_parameters(None), {})

    def test_six_paths_and_bypass_off(self):
        for s, shape in enumerate(('velocity_scurve', 'single_position_polynomial')):
            for e, execution in enumerate(('velocity', 'position', 'attitude')):
                cfg=self.config();cfg.update(shape=shape, execution=execution)
                p=profile_parameters(cfg)
                self.assertEqual(p['hlCommander.pRelShape'], s)
                self.assertEqual(p['hlCommander.pRelExec'], e)
                self.assertEqual(p['hlCommander.pRelLite'], 0)
                self.assertEqual(p['hlCommander.pRelAdapt'], 0)

    def test_invalid_rejected_before_packets(self):
        for key, value in [('shape', 'typo'), ('tail_s', float('nan')), ('tail_s', True),
                           ('single_s', 10), ('feedback', 'truth'), ('handoff', 'moving')]:
            cfg=self.config();cfg[key]=value
            with self.assertRaises(ValueError):profile_parameters(cfg)
        cfg=self.config();cfg['replan']=True
        with self.assertRaises(ValueError):profile_parameters(cfg)
