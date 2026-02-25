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

        # =====================================================================
        # TODO (Step 6): Define the state transition matrix F
        # =====================================================================
        # This matrix describes how the state evolves over one time step.
        # Using a constant velocity model:
        #   x_new  = x + vx * dt
        #   y_new  = y + vy * dt
        #   vx_new = vx
        #   vy_new = vy
        #
        # Fill in the 4x4 matrix:
        self.F = np.array([
            # [?, ?, ?, ?],
            # [?, ?, ?, ?],
            # [?, ?, ?, ?],
            # [?, ?, ?, ?],
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

        # =====================================================================
        # Measurement matrix (Step 9): We measure position only
        # =====================================================================
        # z = [x_measured, y_measured]
        # H extracts position from the state vector
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

        =====================================================================
        TODO (Step 7): Implement the predict step
        =====================================================================
        Two equations:
          1. State prediction:      x = F @ x
          2. Covariance prediction: P = F @ P @ F.T + Q
        """
        # State prediction: x = F * x
        # self.x = ???

        # Covariance prediction: P = F * P * F^T + Q
        # self.P = ???
        pass

    def update(self, z_x: float, z_y: float):
        """
        Update step: Correct the prediction using a measurement.

        This step decreases uncertainty because we get new information
        about where the robot actually is.

        =====================================================================
        TODO (Step 10): Implement the update step
        =====================================================================
        Four equations:
          1. Innovation:            y = z - H @ x
          2. Innovation covariance: S = H @ P @ H.T + R
          3. Kalman gain:           K = P @ H.T @ inv(S)
          4. State update:          x = x + K @ y
          5. Covariance update:     P = (I - K @ H) @ P

        Args:
            z_x: Measured x position
            z_y: Measured y position
        """
        # Measurement vector
        z = np.array([[z_x], [z_y]])

        # Innovation (measurement residual)
        # y = z - H @ x
        # y = ???

        # Innovation covariance
        # S = H @ P @ H.T + R
        # S = ???

        # Kalman gain
        # K = P @ H.T @ inv(S)
        # K = ???

        # State update
        # self.x = x + K @ y
        # self.x = ???

        # Covariance update
        # self.P = (I - K @ H) @ P
        # self.P = ???
        pass

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
