"""
Linear Kalman Filter Implementation

This module implements the standard Linear Kalman Filter for state estimation.
It's designed to be educational and easy to understand.

The Kalman Filter operates in two steps:
1. Predict: Use the motion model to predict the next state
2. Update: Use sensor measurements to correct the prediction

State: x = [x, y, vx, vy]^T  (position and velocity in 2D)

In our robot:
  - PREDICT runs on every odometry callback (~30 Hz) — fast, smooth, drifts
  - UPDATE runs on every GPS callback (~1 Hz) — slow, noisy, no drift
"""

import numpy as np
from typing import Tuple, Optional


class LinearKalmanFilter:
    """
    Linear Kalman Filter for 2D robot localization.

    Fuses fast odometry (predict) with slow GPS (update) to estimate
    the robot's position and velocity.

    State vector: [x, y, vx, vy]
    - x, y: position in meters
    - vx, vy: velocity in m/s

    Example usage:
        kf = LinearKalmanFilter(dt=0.1)
        kf.initialize(x=0.0, y=0.0)

        # On every odom callback (fast):
        kf.predict()

        # On every GPS callback (slow):
        kf.update(gps_x, gps_y)

        x, y, vx, vy = kf.get_state()
    """

    def __init__(
        self,
        dt: float = 0.1,
        process_noise: float = 0.1,
        measurement_noise: float = 2.0,
    ):
        """
        Initialize the Kalman Filter.

        Args:
            dt: Time step in seconds
            process_noise: Process noise std (motion uncertainty)
            measurement_noise: Measurement noise std (GPS noise in meters)
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
        # Measurement matrix (Step 9): GPS measures position only
        # =====================================================================
        # z = [x_gps, y_gps]
        # H extracts position from the state vector
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        # Measurement noise covariance (GPS noise)
        # Higher values = less trust in GPS measurements
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
        """Initialize the filter with a known state."""
        self.x = np.array([[x], [y], [vx], [vy]])
        self.P = np.eye(4) * initial_uncertainty

    def predict(self):
        """
        Prediction step: Estimate the next state based on motion model.

        This step INCREASES uncertainty because we're less sure about
        where the robot is after it moves.

        Called on every odometry callback (~30 Hz).

        =====================================================================
        TODO (Step 7): Implement the predict step
        =====================================================================
        Two equations:
          1. State prediction:      x = F @ x
          2. Covariance prediction: P = F @ P @ F.T + Q
        """
        # State prediction: x = F @ x
        # self.x = ???

        # Covariance prediction: P = F @ P @ F.T + Q
        # self.P = ???
        pass

    def update(self, z_x: float, z_y: float):
        """
        Update step: Correct the prediction using a GPS measurement.

        This step DECREASES uncertainty because we get new information
        about where the robot actually is.

        Called on every GPS callback (~1 Hz).

        =====================================================================
        TODO (Step 10): Implement the update step
        =====================================================================
        Five equations:
          1. Innovation:            y = z - H @ x
          2. Innovation covariance: S = H @ P @ H.T + R
          3. Kalman gain:           K = P @ H.T @ inv(S)
          4. State update:          x = x + K @ y
          5. Covariance update:     P = (I - K @ H) @ P

        Args:
            z_x: GPS x position (meters, local frame)
            z_y: GPS y position (meters, local frame)
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
        """Get current state estimate: (x, y, vx, vy)."""
        return (
            float(self.x[0, 0]),
            float(self.x[1, 0]),
            float(self.x[2, 0]),
            float(self.x[3, 0]),
        )

    def get_covariance(self) -> np.ndarray:
        """Get the 4x4 state covariance matrix."""
        return self.P.copy()

    def get_position_uncertainty(self) -> Tuple[float, float]:
        """Get position standard deviations: (sigma_x, sigma_y)."""
        return np.sqrt(self.P[0, 0]), np.sqrt(self.P[1, 1])
