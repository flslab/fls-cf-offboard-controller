import numpy as np


class VelocityKalmanFilter:
    def __init__(self, dt, process_noise=10, measurement_noise=0.001**2):
        # Time step
        self.dt = dt

        # State vector [position, velocity]
        self.x = np.zeros((2, 1))

        # State transition matrix (Constant Velocity Model)
        self.F = np.array([[1, self.dt],
                           [0, 1]])

        # Measurement matrix (We only measure position)
        self.H = np.array([[1, 0]])

        # Covariance matrix (Uncertainty of our state estimate)
        self.P = np.eye(2) * 1000  # Initialize with high uncertainty

        # Process noise covariance (Q) - How much can the velocity change?
        # A common model for Q assuming acceleration is a random noise
        # A higher process noise means more freedom for velocity change
        G = np.array([[0.5 * self.dt ** 2],
                      [self.dt]])
        self.Q = G @ G.T * process_noise

        # Measurement noise covariance (R) - How noisy is the mocap? vicon is sub-mm accurate, so 0.001
        self.R = np.array([[measurement_noise]])

    def update(self, measurement, return_pos=False):
        # Predict the next state
        self.x = self.F @ self.x
        # Predict the next covariance
        self.P = self.F @ self.P @ self.F.T + self.Q

        # Calculate the Kalman Gain
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # Update the state with the new measurement
        z = np.array([[measurement]])
        y = z - (self.H @ self.x)  # Measurement residual
        self.x = self.x + (K @ y)

        # Update the covariance
        I = np.eye(2)
        self.P = (I - K @ self.H) @ self.P

        if return_pos:
            return self.x[0, 0], self.x[1, 0]  # Return position, velocity

        return self.x[1, 0]  # Return only velocity


if __name__ == '__main__':
    # --- Example Usage ---
    # dt = 0.01 (100Hz mocap), Q_variance = 10, R_variance = 0.01
    kf = VelocityKalmanFilter(dt=0.01, process_noise=10.0, measurement_noise=0.01)

    # Simulated noisy mocap position loop
    # for pos in mocap_data:
    #     smooth_pos, smooth_vel = kf.update(pos)