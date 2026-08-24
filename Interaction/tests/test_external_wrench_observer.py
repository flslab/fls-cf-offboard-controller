import math
import unittest

import numpy as np

from Interaction.external_wrench_observer import (
    ExternalWrenchObserver,
    MotorWrenchModel,
    body_z_world,
    quaternion_to_euler,
)


def yaw_quaternion(yaw):
    return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]


class ExternalWrenchObserverTests(unittest.TestCase):
    def test_quaternion_and_body_axis_conventions(self):
        np.testing.assert_allclose(quaternion_to_euler(yaw_quaternion(math.pi / 2)), [0, 0, math.pi / 2])
        # Positive pitch produces positive world-X thrust at zero yaw.
        self.assertGreater(body_z_world(0, math.radians(10), 0)[0], 0)
        # Positive roll produces negative world-Y thrust at zero yaw.
        self.assertLess(body_z_world(math.radians(10), 0, 0)[1], 0)

    def test_hover_motor_model(self):
        model = MotorWrenchModel(hover_pwm=28000, hover_voltage=8.0)
        linear, angular = model.expected_accelerations([0, 0, 0], [28000] * 4, 8.0)
        np.testing.assert_allclose(linear, np.zeros(3), atol=1e-9)
        np.testing.assert_allclose(angular, np.zeros(3), atol=1e-9)

    def test_estimates_force_and_yaw_torque_from_model_residual(self):
        mass = 0.17
        inertia = [0.002, 0.002, 0.003]
        observer = ExternalWrenchObserver(
            mass=mass,
            inertia=inertia,
            position_measurement_std=[0.001] * 3,
            orientation_measurement_std=[0.002] * 3,
            linear_disturbance_process_std=[0.5] * 3,
            angular_disturbance_process_std=[0.5] * 3,
        )
        dt = 0.01
        external_accel = np.array([0.8, -0.4, 0.3])
        external_angular_accel = np.array([0.2, -0.1, 0.5])
        position = np.zeros(3)
        velocity = np.zeros(3)
        angles = np.zeros(3)
        rates = np.zeros(3)
        estimate = None
        for index in range(500):
            velocity += external_accel * dt
            position += velocity * dt
            rates += external_angular_accel * dt
            angles += rates * dt
            # The synthetic attitude remains small enough that Euler composition
            # is adequately represented by independent-axis test quaternions;
            # torque assertion focuses on yaw.
            estimate = observer.update(
                position,
                yaw_quaternion(angles[2]),
                expected_linear_acceleration=[0, 0, 0],
                expected_angular_acceleration=[0, 0, 0],
                timestamp=index * dt,
            )
        np.testing.assert_allclose(estimate.external_force, mass * external_accel, atol=0.025)
        self.assertAlmostEqual(
            estimate.external_torque[2], inertia[2] * external_angular_accel[2], delta=3e-4
        )

    def test_unwraps_yaw_across_pi(self):
        observer = ExternalWrenchObserver(mass=0.17, inertia=[0.002] * 3)
        first = observer.update([0, 0, 1], yaw_quaternion(math.radians(179)), [0, 0, 0], [0, 0, 0], 0)
        second = observer.update([0, 0, 1], yaw_quaternion(math.radians(-179)), [0, 0, 0], [0, 0, 0], 0.01)
        self.assertGreater(second.orientation_rpy[2], first.orientation_rpy[2])
        self.assertLess(second.orientation_rpy[2] - first.orientation_rpy[2], math.radians(10))


if __name__ == "__main__":
    unittest.main()
