import unittest

import numpy as np

from Interaction.admittance_controller import AdmittanceController3DYaw
from Interaction.external_wrench_observer import WrenchEstimate
from Interaction.wrench_contact_detector import ContactChannelDetector, WrenchContactDetector
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

    def test_roll_pitch_is_independent_and_diagnostic(self):
        common = dict(
            covariance_floor=[1e-4], component_thresholds=[1e-3],
            confidence_sigma=1.0, onset_evidence_s=0.01,
        )
        detector = WrenchContactDetector(
            translation=dict(component_thresholds=[0.1] * 3, covariance_floor=[0.01] * 3),
            yaw=common,
            roll_pitch=dict(
                component_thresholds=[1e-3, 1e-3], covariance_floor=[1e-4, 1e-4],
                confidence_sigma=1.0, onset_evidence_s=0.01,
            ),
        )
        estimate = WrenchEstimate(
            timestamp=0.0,
            position=np.zeros(3), velocity=np.zeros(3),
            orientation_rpy=np.zeros(3), angular_velocity=np.zeros(3),
            external_force=np.zeros(3), external_torque=np.array([0.01, 0, 0]),
            force_covariance=np.eye(3) * 1e-5,
            torque_covariance=np.eye(3) * 1e-8,
            position_innovation=np.zeros(3), orientation_innovation=np.zeros(3),
            position_nis=0, orientation_nis=0, measurement_rejected=False,
        )
        detector.update(estimate)
        state = detector.update(WrenchEstimate(**{**estimate.__dict__, "timestamp": 0.02}))
        self.assertTrue(state.roll_pitch.active)
        self.assertFalse(state.translation.active)
        self.assertFalse(state.yaw.active)


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

    def test_roll_pitch_detection_never_routes_to_response(self):
        contacts = WrenchContactState(
            translation=self.decision(False),
            yaw=self.decision(False),
            roll_pitch=self.decision(True),
        )
        force, yaw_torque = WrenchInteractionPipeline._response_inputs(
            self.estimate(), contacts
        )
        np.testing.assert_array_equal(force, np.zeros(3))
        self.assertEqual(yaw_torque, 0.0)

    def test_only_xyz_force_and_yaw_torque_route_to_response(self):
        contacts = WrenchContactState(
            translation=self.decision(True),
            yaw=self.decision(True),
            roll_pitch=self.decision(True),
        )
        force, yaw_torque = WrenchInteractionPipeline._response_inputs(
            self.estimate(), contacts
        )
        np.testing.assert_array_equal(force, [0.2, -0.1, 0.3])
        self.assertEqual(yaw_torque, 0.03)

    def test_active_mode_requires_identified_angular_motor_model(self):
        with self.assertRaisesRegex(ValueError, 'angular_accel_scale'):
            WrenchInteractionPipeline({'shadow_mode': False})

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


if __name__ == "__main__":
    unittest.main()
