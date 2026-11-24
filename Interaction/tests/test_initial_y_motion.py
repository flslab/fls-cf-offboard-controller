import unittest
import numpy as np

from Interaction.initial_y_motion import InitialYMotion, validate_initial_motion


class InitialYMotionTests(unittest.TestCase):
    def test_disabled_default_keeps_every_phase_unchanged(self):
        motion = InitialYMotion([.2, -.3, 1.], {})
        for phase in ('prepare', 'ready', 'contact', 'coast', 'capture', 'ready'):
            target, status = motion.update(phase, 1.)
            self.assertIsNone(target)
            self.assertEqual(status, {})

    def test_waits_for_ready_and_uses_only_ready_sample_clock(self):
        motion = InitialYMotion([.2, -.3, 1.], {'enabled': True})
        for stamp in (1., 5., 10.):
            self.assertIsNone(motion.update('prepare', stamp)[0])
            self.assertIsNone(motion.started)
        target, status = motion.update('ready', 20.)
        np.testing.assert_allclose(target, [.2, -.3, 1.])
        self.assertEqual(status['initial_motion_reference_speed_m_s'], 0.)

    def test_world_y_trajectory_ramps_cruises_and_stops_at_distance(self):
        motion = InitialYMotion([.2, -.3, 1.], dict(enabled=True, speed_m_s=.03,
                                ramp_s=.1, max_distance_m=.006))
        targets=[];speeds=[]
        for stamp in np.arange(0., .311, .01):
            target, status = motion.update('ready', float(stamp))
            if target is not None:targets.append(target)
            speeds.append(status['initial_motion_reference_speed_m_s'])
        targets=np.array(targets)
        np.testing.assert_allclose(targets[:,0], .2)
        np.testing.assert_allclose(targets[:,2], 1.)
        self.assertTrue(np.all(np.diff(targets[:,1]) >= -1e-12))
        self.assertLessEqual(np.max(targets[:,1]), -.294+1e-12)
        self.assertAlmostEqual(targets[-1,1], -.294)
        self.assertAlmostEqual(max(speeds), .03)
        self.assertEqual(speeds[0], 0.)
        self.assertEqual(speeds[-1], 0.)
        self.assertEqual(motion.finished_reason, 'max_distance')
        self.assertIsNone(motion.update('ready', 1.)[0])

    def test_short_distance_reduces_peak_speed_instead_of_overshooting(self):
        motion=InitialYMotion([0., 0., 1.], dict(enabled=True, max_distance_m=.006))
        self.assertAlmostEqual(motion.speed, .006)
        for stamp in np.arange(0., 2.01, .05):
            target, _=motion.update('ready', float(stamp))
            self.assertLessEqual(target[1], .006+1e-12)
        self.assertAlmostEqual(motion.displacement, .006)

    def test_first_contact_permanently_retires_motion_even_during_ramp(self):
        motion=InitialYMotion([0., 0., 1.], {'enabled': True})
        motion.update('ready', 0.)
        target,_=motion.update('ready', .05)
        self.assertGreater(target[1], 0.)
        last_distance=motion.displacement
        self.assertIsNone(motion.update('contact', .06)[0])
        for phase in ('contact', 'coast', 'capture', 'grace', 'prepare', 'ready', 'contact', 'ready'):
            self.assertIsNone(motion.update(phase, 2.)[0])
            self.assertEqual(motion.displacement, last_distance)
        self.assertEqual(motion.finished_reason, 'first_contact')

    def test_contact_before_motion_start_also_prevents_later_start(self):
        motion=InitialYMotion([0., 0., 1.], {'enabled': True})
        motion.update('contact', 0.)
        self.assertIsNone(motion.update('ready', 1.)[0])

    def test_boundary_margin_limits_reference_and_never_commands_backwards(self):
        motion=InitialYMotion([.2, .77, 1.], {'enabled': True}, y_max=1.)
        for stamp in np.arange(0., 2.01, .05):
            target,_=motion.update('ready', float(stamp))
            self.assertGreaterEqual(target[1], .77)
            self.assertLessEqual(target[1], .8+1e-12)
        self.assertEqual(motion.finished_reason, 'boundary_margin')
        blocked=InitialYMotion([.2, .9, 1.], {'enabled': True}, y_max=1.)
        target,status=blocked.update('ready', 0.)
        np.testing.assert_allclose(target, [.2, .9, 1.])
        self.assertEqual(status['initial_motion_finished_reason'], 'boundary_margin')

    def test_duplicate_samples_do_not_advance_and_bad_clocks_fail(self):
        motion=InitialYMotion([0., 0., 1.], {'enabled': True})
        motion.update('ready', 10.)
        target,_=motion.update('ready', 10.05)
        repeated,_=motion.update('ready', 10.05)
        np.testing.assert_array_equal(target, repeated)
        for stamp in (10.04, 10.3, float('nan')):
            with self.subTest(stamp=stamp), self.assertRaises(ValueError):
                motion.update('ready', stamp)

    def test_invalid_configuration_fails_before_commands(self):
        for value in (None, [], True):
            with self.assertRaises(ValueError):validate_initial_motion(value)
        for key,value in [('enabled','true'),('speed_m_s',.11),('speed_m_s',0.),
                          ('ramp_s',False),('max_distance_m',float('nan')),
                          ('boundary_margin_m',-1.),('unknown',1.)]:
            with self.subTest(key=key,value=value), self.assertRaises(ValueError):
                validate_initial_motion({key:value})


if __name__=='__main__':
    unittest.main()
