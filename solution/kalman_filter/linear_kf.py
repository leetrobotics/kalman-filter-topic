"""
Linear Kalman Filter Implementation

This module implements the standard Linear Kalman Filter for state estimation.
It's designed to be educational and easy to understand.

The Kalman Filter operates in two steps:
1. Predict: Use the motion model to predict the next state
2. Update: Use sensor measurements to correct the prediction

State: x = [x, y, vx, vy]^T  (position and velocity in 2D)
"""

import numpy as np
from typing import Tuple, Optional


class LinearKalmanFilter:
    """
    Linear Kalman Filter for 2D robot localization.

    This filter estimates the robot's position and velocity from noisy
    odometry measurements.

    State vector: [x, y, vx, vy]
    - x, y: position in meters
    - vx, vy: velocity in m/s

    Example usage:
        kf = LinearKalmanFilter(dt=0.1)

        # Initialize with known position
        kf.initialize(x=0.0, y=0.0)

        # In your control loop:
        kf.predict()
        kf.update(measured_x, measured_y)

        # Get the estimated state
        x, y, vx, vy = kf.get_state()
        covariance = kf.get_covariance()
    """

    def __init__(
        self,
        dt: float = 0.1,
        process_noise: float = 0.1,
        measurement_noise: float = 0.5,
    ):
        """
        Initialize the Kalman Filter.

        Args:
            dt: Time step in seconds
            process_noise: Process noise standard deviation (motion uncertainty)
            measurement_noise: Measurement noise standard deviation (sensor noise)
        """
        self.dt = dt

        # State vector: [x, y, vx, vy]
        self.x = np.zeros((4, 1))

        # State covariance matrix (uncertainty)
        self.P = np.eye(4) * 1.0

        # State transition matrix (motion model)
        # x_new = x + vx * dt
        # y_new = y + vy * dt
        # vx_new = vx (constant velocity model)
        # vy_new = vy
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # Process noise covariance (motion uncertainty)
        # Higher values = less trust in motion model
        q = process_noise
        self.Q = np.array([
            [dt**4/4, 0, dt**3/2, 0],
            [0, dt**4/4, 0, dt**3/2],
            [dt**3/2, 0, dt**2, 0],
            [0, dt**3/2, 0, dt**2]
        ]) * q**2

        # Measurement matrix (we measure position only)
        # z = [x_measured, y_measured]
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        # Measurement noise covariance (sensor noise)
        # Higher values = less trust in measurements
        r = measurement_noise
        self.R = np.eye(2) * r**2

        # Identity matrix for update step
        self.I = np.eye(4)

    def initialize(
        self,
        x: float = 0.0,
        y: float = 0.0,
        vx: float = 0.0,
        vy: float = 0.0,
        initial_uncertainty: float = 1.0,
    ):
        """
        Initialize the filter with a known state.

        Args:
            x, y: Initial position
            vx, vy: Initial velocity
            initial_uncertainty: Initial uncertainty (diagonal of P)
        """
        self.x = np.array([[x], [y], [vx], [vy]])
        self.P = np.eye(4) * initial_uncertainty

    def predict(self, control_input: Optional[np.ndarray] = None):
        """
        Prediction step: Estimate the next state based on motion model.

        This step increases uncertainty because we're less sure about
        where the robot is after it moves.

        Args:
            control_input: Optional control input (not used in constant velocity model)
        """
        # State prediction: x = F * x
        self.x = self.F @ self.x

        # Covariance prediction: P = F * P * F^T + Q
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z_x: float, z_y: float):
        """
        Update step: Correct the prediction using a measurement.

        This step decreases uncertainty because we get new information
        about where the robot actually is.

        Args:
            z_x: Measured x position
            z_y: Measured y position
        """
        # Measurement vector
        z = np.array([[z_x], [z_y]])

        # Innovation (measurement residual)
        # y = z - H * x (difference between measurement and prediction)
        y = z - self.H @ self.x

        # Innovation covariance
        # S = H * P * H^T + R
        S = self.H @ self.P @ self.H.T + self.R

        # Kalman gain
        # K = P * H^T * S^-1
        # This determines how much we trust the measurement vs prediction
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # State update
        # x = x + K * y
        self.x = self.x + K @ y

        # Covariance update (Joseph form for numerical stability)
        # P = (I - K * H) * P
        self.P = (self.I - K @ self.H) @ self.P

    def get_state(self) -> Tuple[float, float, float, float]:
        """
        Get the current state estimate.

        Returns:
            Tuple of (x, y, vx, vy)
        """
        return (
            float(self.x[0, 0]),
            float(self.x[1, 0]),
            float(self.x[2, 0]),
            float(self.x[3, 0]),
        )

    def get_position(self) -> Tuple[float, float]:
        """
        Get just the position estimate.

        Returns:
            Tuple of (x, y)
        """
        return float(self.x[0, 0]), float(self.x[1, 0])

    def get_covariance(self) -> np.ndarray:
        """
        Get the state covariance matrix.

        The diagonal elements represent the variance (uncertainty squared)
        of each state variable.

        Returns:
            4x4 covariance matrix
        """
        return self.P.copy()

    def get_position_uncertainty(self) -> Tuple[float, float]:
        """
        Get the standard deviation of position estimates.

        This can be used to draw uncertainty ellipses.

        Returns:
            Tuple of (sigma_x, sigma_y)
        """
        return np.sqrt(self.P[0, 0]), np.sqrt(self.P[1, 1])


# =============================================================================
# Example: Simple 1D Position Tracking
# =============================================================================
def example_1d_tracking():
    """
    Demonstrate Kalman filtering with a simple 1D example.

    A robot moves in a straight line. We have:
    - Noisy velocity commands
    - Noisy position measurements

    The Kalman filter fuses these to get a better estimate.
    """
    import matplotlib.pyplot as plt

    # Simulation parameters
    dt = 0.1
    num_steps = 100
    true_velocity = 1.0  # m/s

    # Noise levels
    process_noise = 0.1   # Motion uncertainty
    measurement_noise = 0.5  # Sensor noise

    # Create filter (using just x and vx for 1D)
    kf = LinearKalmanFilter(dt=dt, process_noise=process_noise, measurement_noise=measurement_noise)
    kf.initialize(x=0.0, y=0.0, vx=true_velocity, vy=0.0)

    # Storage for plotting
    true_positions = []
    measured_positions = []
    estimated_positions = []
    uncertainties = []

    true_x = 0.0

    for step in range(num_steps):
        # True motion (ground truth)
        true_x += true_velocity * dt
        true_positions.append(true_x)

        # Noisy measurement
        measured_x = true_x + np.random.normal(0, measurement_noise)
        measured_positions.append(measured_x)

        # Kalman filter
        kf.predict()
        kf.update(measured_x, 0.0)  # y=0 for 1D

        est_x, _, _, _ = kf.get_state()
        sigma_x, _ = kf.get_position_uncertainty()

        estimated_positions.append(est_x)
        uncertainties.append(sigma_x)

    # Plot results
    time = np.arange(num_steps) * dt

    plt.figure(figsize=(12, 6))

    plt.subplot(2, 1, 1)
    plt.plot(time, true_positions, 'g-', label='True Position', linewidth=2)
    plt.plot(time, measured_positions, 'r.', label='Measurements', alpha=0.5)
    plt.plot(time, estimated_positions, 'b-', label='Kalman Estimate', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.title('1D Kalman Filter: Position Tracking')
    plt.legend()
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(time, uncertainties, 'b-', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Position Uncertainty (m)')
    plt.title('Position Uncertainty Over Time')
    plt.grid(True)

    plt.tight_layout()
    plt.savefig('kalman_1d_example.png', dpi=150)
    plt.show()

    print("Example complete! Check kalman_1d_example.png")


if __name__ == "__main__":
    example_1d_tracking()
