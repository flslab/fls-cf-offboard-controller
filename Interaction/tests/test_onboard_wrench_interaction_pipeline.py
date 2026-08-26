import unittest

import numpy as np

from Interaction.onboard_wrench_interaction_pipeline import (
    FiniteWindowMomentumForceEstimator,
    OnboardMomentumWrenchObserver,
    OnboardMomentumWrenchPipeline,
)


class FiniteWindowMomentumForceEstimatorTests(unittest.TestCase):
    def test_constant_velocity_inertia_is_not_external_force(self):
        estimator = FiniteWindowMomentumForceEstimator(
            mass=0.17, window_s=0.08, minimum_window_s=0.05
        )
        estimate = None
        for index in range(20):
            estimate = estimator.update(
                velocity=[0.8, -0.3, 0.1],
                expected_linear_acceleration=[0.0, 0.0, 0.0],
                timestamp=index * 0.01,
            )

        self.assertTrue(estimate.ready)
        np.testing.assert_allclose(estimate.external_force, np.zeros(3), atol=1e-12)
        np.testing.assert_allclose(
            estimate.predicted_velocity, [0.8, -0.3, 0.1], atol=1e-12
        )

    def test_motor_explained_acceleration_is_not_external_force(self):
        estimator = FiniteWindowMomentumForceEstimator(
            mass=0.17, window_s=0.08, minimum_window_s=0.05
        )
        acceleration = np.array([1.2, -0.4, 0.3])
        velocity = np.zeros(3)
        estimate = estimator.update(velocity, acceleration, 0.0)
        for index in range(1, 20):
            velocity += acceleration * 0.01
            estimate = estimator.update(
                velocity, acceleration, index * 0.01
            )

        self.assertTrue(estimate.ready)
        np.testing.assert_allclose(estimate.external_force, np.zeros(3), atol=1e-12)

    def test_release_decays_without_opposite_force_tail(self):
        mass = 0.17
        estimator = FiniteWindowMomentumForceEstimator(
            mass=mass, window_s=0.08, minimum_window_s=0.05
        )
        applied_force = np.array([0.12, 0.0, 0.0])
        velocity = np.zeros(3)
        estimator.update(velocity, np.zeros(3), 0.0)

        estimate = None
        for index in range(1, 16):
            velocity += applied_force / mass * 0.01
            estimate = estimator.update(
                velocity, np.zeros(3), index * 0.01
            )
        self.assertAlmostEqual(estimate.external_force[0], 0.12, places=10)

        release_forces = []
        for index in range(16, 31):
            estimate = estimator.update(
                velocity, np.zeros(3), index * 0.01
            )
            if estimate.ready:
                release_forces.append(float(estimate.external_force[0]))

        self.assertGreater(release_forces[0], 0.0)
        self.assertGreaterEqual(min(release_forces), -1e-12)
        self.assertAlmostEqual(release_forces[-1], 0.0, places=10)


class OnboardMomentumObserverTests(unittest.TestCase):
    def test_converges_to_constant_external_force_and_torque(self):
        mass = 0.17
        inertia = np.array([0.002, 0.002, 0.003])
        observer = OnboardMomentumWrenchObserver(
            mass=mass,
            inertia=inertia,
            linear_gain=[12.0] * 3,
            angular_gain=[16.0] * 3,
        )
        dt = 0.01
        external_force = np.array([0.12, -0.06, 0.08])
        external_torque = np.array([0.0010, -0.0005, 0.0015])
        velocity = np.zeros(3)
        angular_velocity = np.zeros(3)
        estimate = None

        for index in range(500):
            velocity += (external_force / mass) * dt
            gyroscopic_torque = np.cross(
                angular_velocity, inertia * angular_velocity
            )
            angular_velocity += (
                (external_torque - gyroscopic_torque) / inertia
            ) * dt
            estimate = observer.update(
                position=np.zeros(3),
                velocity=velocity,
                attitude_rpy=np.zeros(3),
                angular_velocity=angular_velocity,
                expected_linear_acceleration=np.zeros(3),
                expected_angular_acceleration=np.zeros(3),
                timestamp=index * dt,
            )

        np.testing.assert_allclose(
            estimate.external_force, external_force, atol=0.004
        )
        np.testing.assert_allclose(
            estimate.external_torque, external_torque, atol=5e-5
        )

    def test_rejects_large_timestamp_gap_without_force_impulse(self):
        observer = OnboardMomentumWrenchObserver(
            mass=0.17, inertia=[0.002, 0.002, 0.003], max_dt_s=0.05
        )
        observer.update(
            [0, 0, 1], [0, 0, 0], [0, 0, 0], [0, 0, 0],
            [0, 0, 0], [0, 0, 0], 0.0,
        )
        estimate = observer.update(
            [0, 0, 1], [1, 0, 0], [0, 0, 0], [0, 0, 0],
            [0, 0, 0], [0, 0, 0], 0.2,
        )
        self.assertTrue(estimate.measurement_rejected)
        np.testing.assert_array_equal(estimate.external_force, np.zeros(3))

    def test_one_step_prediction_uses_previous_actuator_state(self):
        mass = 0.17
        observer = OnboardMomentumWrenchObserver(
            mass=mass, inertia=[0.002, 0.002, 0.003]
        )
        zero = [0.0, 0.0, 0.0]

        observer.update(
            zero, zero, zero, zero,
            [2.0, 0.0, 0.0], zero, 0.0,
        )
        first = observer.update(
            zero, [0.02, 0.0, 0.0], zero, zero,
            [-1.0, 0.0, 0.0], zero, 0.01,
        )
        second = observer.update(
            zero, [0.01, 0.0, 0.0], zero, zero,
            zero, zero, 0.02,
        )

        np.testing.assert_allclose(first.position_innovation, zero, atol=1e-12)
        np.testing.assert_allclose(second.position_innovation, zero, atol=1e-12)
        np.testing.assert_allclose(first.external_force, zero, atol=1e-12)
        np.testing.assert_allclose(second.external_force, zero, atol=1e-12)


class OnboardMomentumPipelineTests(unittest.TestCase):
    def test_yaw_command_model_predicts_yaw_acceleration(self):
        pipeline = OnboardMomentumWrenchPipeline({
            'motor_model': {
                'yaw_command_model': {
                    'enabled': True,
                    'accel_per_command': 0.002,
                    'damping_per_s': 0.5,
                    'bias_rad_s2': 0.1,
                },
            },
        })
        output = pipeline.update(
            position=[0, 0, 1], velocity=[0, 0, 0],
            attitude_rpy=[0, 0, 0], angular_velocity=[0, 0, 0.2],
            motor_pwm=[30000] * 4, battery_voltage=8.0,
            yaw_control_command=100.0, timestamp=0.0,
        )
        self.assertAlmostEqual(output.expected_angular_acceleration[2], 0.2)

    def test_calibrates_from_onboard_state_without_mocap_quaternion(self):
        pipeline = OnboardMomentumWrenchPipeline({
            'observer_settle_s': 0.0,
            'bias_calibration_s': 0.02,
            'minimum_bias_samples': 2,
            'motor_model': {'hover_pwm': 30000, 'hover_voltage': 8.0},
            'impulse_estimator': {
                'window_s': 0.02,
                'minimum_window_s': 0.01,
            },
        })
        args = dict(
            position=[0, 0, 1],
            velocity=[0, 0, 0],
            attitude_rpy=[0, 0, 0],
            angular_velocity=[0, 0, 0],
            motor_pwm=[30000] * 4,
            battery_voltage=8.0,
        )
        first = pipeline.update(**args, timestamp=0.0)
        second = pipeline.update(**args, timestamp=0.01)
        third = pipeline.update(**args, timestamp=0.03)
        fourth = pipeline.update(**args, timestamp=0.04)
        self.assertFalse(first.calibrated)
        self.assertIsNone(second.contacts)
        self.assertFalse(third.calibrated)
        self.assertTrue(fourth.calibrated)
        self.assertIsNotNone(fourth.contacts)


if __name__ == '__main__':
    unittest.main()
