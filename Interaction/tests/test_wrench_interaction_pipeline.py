import unittest

import numpy as np

from Interaction.admittance_controller import AdmittanceController3DYaw
from Interaction.external_wrench_observer import WrenchEstimate
from Interaction.wrench_contact_detector import ContactChannelDetector
from Interaction.wrench_contact_detector import ContactDecision, WrenchContactState
from Interaction.wrench_interaction_pipeline import WrenchInteractionPipeline


class ContactDetectorTests(unittest.TestCase):
    def test_requires_persistent_statistically_significant_force(self):
        detector = ContactChannelDetector(
            component_thresholds=[0.05, 0.05, 0.08],
            covariance_floor=[0.005, 0.005, 0.008],
            confidence_sigma=2.0,
            onset_evidence_s=0.03,
            release_time_s=0.05,
        )
        covariance = np.eye(3) * 1e-6
        # One high sample is not enough.
        self.assertFalse(detector.update([0.1, 0, 0], covariance, 0.0).started)
        self.assertFalse(detector.update([0, 0, 0], covariance, 0.01).active)

        started = False
        for index in range(2, 12):
            started |= detector.update([0.12, 0, 0], covariance, index * 0.01).started
        self.assertTrue(started)
        self.assertTrue(detector.active)

        ended = False
        for index in range(12, 20):
            ended |= detector.update([0, 0, 0], covariance, index * 0.01).ended
        self.assertTrue(ended)

        detector.reset(0.20)
        self.assertFalse(detector.active)
        self.assertEqual(detector.evidence, 0.0)

    def test_disabled_channel_never_starts(self):
        detector = ContactChannelDetector(
            component_thresholds=[0.05],
            covariance_floor=[0.005],
            enabled=False,
        )
        decision = detector.update([100.0], np.eye(1) * 1e-9, 0.0)
        self.assertFalse(decision.active)
        self.assertFalse(decision.started)
        self.assertEqual(decision.normalized_magnitude, 0.0)

    def test_projected_release_ignores_side_and_vertical_force(self):
        detector = ContactChannelDetector(
            component_thresholds=[0.08, 0.08, 0.12],
            covariance_floor=[0.005, 0.005, 0.008],
            confidence_sigma=2.0,
            onset_evidence_s=0.01,
            release_time_s=0.15,
            release_ratio=0.55,
            release_projection_axes=[0, 1],
            release_direction_min_norm=0.02,
        )
        covariance = np.eye(3) * 1e-6
        decision = None
        for index in range(4):
            decision = detector.update(
                [0.0, 0.20, 0.0],
                covariance,
                index * 0.01,
                release_direction_candidate=[0.0, 0.10, 0.08],
            )
        self.assertTrue(decision.active)
        self.assertEqual(decision.release_direction, (0.0, 1.0, 0.0))
        self.assertEqual(decision.release_direction_source, "force")

        ended_decision = None
        for index in range(4, 21):
            # Large X and Z force remains, but force along locked +Y is zero.
            decision = detector.update(
                [0.20, 0.0, 0.20],
                covariance,
                index * 0.01,
                release_direction_candidate=[0.10, 0.0, 0.0],
            )
            if decision.ended:
                ended_decision = decision
                break
        self.assertIsNotNone(ended_decision)
        self.assertAlmostEqual(ended_decision.release_projected_value, 0.0)
        self.assertAlmostEqual(
            ended_decision.release_projection_normalized, 0.0
        )

        # Large residual feedback must not immediately retrigger XYZ onset.
        for index in range(21, 35):
            decision = detector.update(
                [0.20, -0.20, 0.20],
                covariance,
                index * 0.01,
                release_direction_candidate=[0.10, -0.10, 0.0],
            )
            self.assertFalse(decision.active)
            self.assertFalse(decision.started)

        # A quiet sample alone does not re-arm while braking is in progress.
        detector.update([0.0, 0.0, 0.0], covariance, 0.35)
        decision = detector.update(
            [0.20, 0.0, 0.0],
            covariance,
            0.36,
            release_direction_candidate=[0.10, 0.0, 0.0],
        )
        self.assertFalse(decision.active)

        # The controller handoff explicitly resets after braking completes.
        detector.reset(0.36)
        restarted = False
        for index in range(37, 43):
            decision = detector.update(
                [0.20, 0.0, 0.0],
                covariance,
                index * 0.01,
                release_direction_candidate=[0.10, 0.0, 0.0],
            )
            restarted |= decision.started
        self.assertTrue(restarted)

    def test_projected_release_prefers_force_over_start_velocity(self):
        detector = ContactChannelDetector(
            component_thresholds=[0.08, 0.08, 0.12],
            covariance_floor=[0.005, 0.005, 0.008],
            confidence_sigma=2.0,
            onset_evidence_s=0.01,
            release_projection_axes=[0, 1],
            release_direction_min_norm=0.02,
        )
        covariance = np.eye(3) * 1e-6
        decision = None
        for index in range(4):
            decision = detector.update(
                [0.0, -0.20, 0.0],
                covariance,
                index * 0.01,
                release_direction_candidate=[0.10, 0.0, 0.0],
            )
        self.assertTrue(decision.active)
        self.assertEqual(decision.release_direction, (0.0, -1.0, 0.0))
        self.assertEqual(decision.release_direction_source, "force")

    def test_large_opposite_projection_counts_as_release_after_dwell(self):
        detector = ContactChannelDetector(
            component_thresholds=[0.08, 0.08, 0.12],
            covariance_floor=[0.005, 0.005, 0.008],
            confidence_sigma=2.0,
            onset_evidence_s=0.01,
            release_time_s=0.15,
            release_ratio=0.55,
            release_projection_axes=[0, 1],
        )
        covariance = np.eye(3) * 1e-6
        decision = None
        for index in range(4):
            decision = detector.update(
                [0.0, 0.20, 0.0],
                covariance,
                index * 0.01,
                release_direction_candidate=[0.0, 0.10, 0.0],
            )
        self.assertTrue(decision.active)

        ended_decision = None
        for index in range(4, 24):
            decision = detector.update(
                [0.0, -0.20, 0.0],
                covariance,
                index * 0.01,
                release_direction_candidate=[0.0, -0.10, 0.0],
            )
            if decision.ended:
                ended_decision = decision
                break
        self.assertIsNotNone(ended_decision)
        self.assertFalse(ended_decision.active)
        self.assertLess(
            ended_decision.release_projection_normalized, 0.0
        )

class AdmittanceTests(unittest.TestCase):
    def make_controller(self):
        return AdmittanceController3DYaw(
            translation_mass=[0.3, 0.3, 0.5],
            translation_damping=[1, 1, 1.5],
            translation_stiffness=[0, 0, 0],
            max_offset=[0.2, 0.2, 0.1],
            max_velocity=[0.3, 0.3, 0.15],
            max_acceleration=[1, 1, 0.5],
            yaw_inertia=0.01,
            yaw_damping=0.05,
            yaw_stiffness=0,
            max_yaw_offset=0.5,
            max_yaw_rate=0.4,
            max_yaw_acceleration=1.0,
        )

    def test_generates_xyz_and_yaw_offsets(self):
        controller = self.make_controller()
        state = None
        for _ in range(100):
            state = controller.step([0.2, -0.1, 0.15], 0.02, 0.01)
        self.assertGreater(state.translation_offset[0], 0)
        self.assertLess(state.translation_offset[1], 0)
        self.assertGreater(state.translation_offset[2], 0)
        self.assertGreater(state.yaw_offset, 0)

    def test_enforces_all_limits(self):
        controller = self.make_controller()
        for _ in range(1000):
            state = controller.step([10, 10, 10], 10, 0.01)
        self.assertTrue(np.all(np.abs(state.translation_offset) <= [0.2, 0.2, 0.1]))
        self.assertTrue(np.all(np.abs(state.translation_velocity) <= [0.3, 0.3, 0.15]))
        self.assertLessEqual(abs(state.yaw_offset), 0.5)
        self.assertLessEqual(abs(state.yaw_rate), 0.4)


class PipelineRoutingTests(unittest.TestCase):
    @staticmethod
    def decision(active):
        return ContactDecision(active, False, False, 0, 0, 0, 0)

    @staticmethod
    def estimate():
        return WrenchEstimate(
            timestamp=0.0,
            position=np.zeros(3), velocity=np.zeros(3),
            orientation_rpy=np.zeros(3), angular_velocity=np.zeros(3),
            external_force=np.array([0.2, -0.1, 0.3]),
            external_torque=np.array([0.01, -0.02, 0.03]),
            force_covariance=np.eye(3), torque_covariance=np.eye(3),
            position_innovation=np.zeros(3), orientation_innovation=np.zeros(3),
            position_nis=0, orientation_nis=0, measurement_rejected=False,
        )

    def test_only_xyz_force_and_yaw_torque_route_to_response(self):
        contacts = WrenchContactState(
            translation=self.decision(True),
            yaw=self.decision(True),
        )
        force, yaw_torque = WrenchInteractionPipeline._response_inputs(
            self.estimate(), contacts
        )
        np.testing.assert_array_equal(force, [0.2, -0.1, 0.3])
        self.assertEqual(yaw_torque, 0.03)

    def test_active_mode_requires_identified_angular_motor_model(self):
        with self.assertRaisesRegex(ValueError, 'angular_accel_scale'):
            WrenchInteractionPipeline({'shadow_mode': False})

    def test_startup_bias_calibration_can_be_skipped(self):
        pipeline = WrenchInteractionPipeline({
            'startup_bias_calibration_enabled': False,
        })

        self.assertTrue(pipeline.calibrated)
        self.assertEqual(pipeline.calibration_samples, 0)
        np.testing.assert_array_equal(pipeline.force_bias, np.zeros(3))
        np.testing.assert_array_equal(pipeline.torque_bias, np.zeros(3))

    def test_active_mode_only_requires_yaw_angular_model(self):
        WrenchInteractionPipeline({
            'shadow_mode': False,
            'motor_model': {'angular_accel_scale': [0.0, 0.0, 1.0]},
        })

    def test_active_mode_accepts_calibrated_yaw_command_model(self):
        WrenchInteractionPipeline({
            'shadow_mode': False,
            'motor_model': {
                'yaw_command_model': {
                    'enabled': True,
                    'accel_per_command': 0.002,
                    'damping_per_s': 0.5,
                    'bias_rad_s2': 0.0,
                },
            },
        })

    def test_active_xyz_mode_does_not_require_yaw_model(self):
        WrenchInteractionPipeline({
            'shadow_mode': False,
            'detection': {'yaw': {'enabled': False}},
        })

    def test_bias_calibration_completes_before_contact_detection(self):
        pipeline = WrenchInteractionPipeline({
            'observer_settle_s': 0.0,
            'bias_calibration_s': 0.02,
            'minimum_bias_samples': 2,
            'motor_model': {'hover_pwm': 30000, 'hover_voltage': 8.0},
        })
        first = pipeline.update([0, 0, 1], [0, 0, 0, 1], [30000] * 4, 8.0, 0.0)
        second = pipeline.update([0, 0, 1], [0, 0, 0, 1], [30000] * 4, 8.0, 0.01)
        third = pipeline.update([0, 0, 1], [0, 0, 0, 1], [30000] * 4, 8.0, 0.03)
        self.assertFalse(first.calibrated)
        self.assertIsNone(second.contacts)
        self.assertTrue(third.calibrated)
        self.assertIsNotNone(third.contacts)

    def test_bias_calibration_restarts_after_motion(self):
        pipeline = WrenchInteractionPipeline({
            'observer_settle_s': 0.0,
            'bias_calibration_s': 0.02,
            'minimum_bias_samples': 2,
        })
        pipeline._start_timestamp = 0.0
        base = self.estimate()

        pipeline._update_bias(base)
        moving = base.__class__(
            **{**base.__dict__, 'timestamp': 0.01,
               'velocity': np.array([0.2, 0.0, 0.0])}
        )
        pipeline._update_bias(moving)
        self.assertEqual(pipeline.calibration_samples, 0)

        for timestamp in (0.02, 0.03, 0.05):
            stationary = base.__class__(
                **{**base.__dict__, 'timestamp': timestamp}
            )
            pipeline._update_bias(stationary)
        self.assertTrue(pipeline.calibrated)


if __name__ == "__main__":
    unittest.main()
