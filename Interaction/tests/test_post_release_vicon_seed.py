import unittest
from types import SimpleNamespace

import numpy as np

from Interaction.Kalman_Filter import VelocityKalmanFilter
from Interaction.post_release_vicon_seed import vicon_velocity_seed_candidate


class ViconVelocitySeedCandidateTests(unittest.TestCase):
    def setUp(self):
        filters = {axis: VelocityKalmanFilter(dt=0.01, process_noise=1.0)
                   for axis in 'xyz'}
        frames = []
        for index in range(12):
            position = np.array([0.0, -0.01 * index, 1.0])
            velocity = [filters[axis].update(pos)
                        for axis, pos in zip('xyz', position)]
            frames.append({
                'host_receive_monotonic_s': 10.0 + 0.01 * index,
                'tvec': position.tolist(), 'vel': velocity,
            })
        self.frames = frames
        self.manager = SimpleNamespace(
            groups={'frames': frames}, group_kfs={'frames': filters})

    def candidate(self, **kwargs):
        return vicon_velocity_seed_candidate(
            self.manager, 'frames', kwargs.get('release_time', 10.115),
            kwargs.get('release_position', [0.0, -0.11, 1.0]))

    def test_recent_candidate_preserves_pi_time_basis_and_covariance_floor(self):
        candidate = self.candidate()
        self.assertAlmostEqual(candidate.pi_frame_receive_monotonic_s, 10.11)
        self.assertAlmostEqual(candidate.pi_release_receive_monotonic_s, 10.115)
        self.assertLess(candidate.velocity_m_s[1], -0.5)
        self.assertTrue(all(value >= 0.08
                            for value in candidate.velocity_std_m_s))

    def test_stale_burst_and_position_mismatch_reject(self):
        with self.assertRaises(ValueError):
            self.candidate(release_time=10.16)
        with self.assertRaises(ValueError):
            self.candidate(release_position=[0.5, -0.11, 1.0])
        self.frames[-1]['host_receive_monotonic_s'] = 10.101
        with self.assertRaises(ValueError):
            self.candidate()

    def test_unsettled_covariance_rejects(self):
        self.manager.group_kfs['frames']['y'].P[1, 1] = 1.0
        with self.assertRaises(ValueError):
            self.candidate()

    def test_kf_advanced_past_latest_frame_rejects(self):
        self.manager.group_kfs['frames']['y'].update(-0.20)
        with self.assertRaises(ValueError):
            self.candidate()


if __name__ == '__main__':
    unittest.main()
