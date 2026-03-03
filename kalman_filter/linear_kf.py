"""
Linear Kalman Filter Implementation

State: x = [x, y, vx, vy]^T  (position and velocity in 2D)

Two steps:
1. Predict: Use motion model to propagate state forward
2. Update: Use GPS measurement to correct the estimate
"""

import numpy as np
from typing import Tuple, Optional


class LinearKalmanFilter:
    """
    Linear Kalman Filter for 2D robot localization.

    Fuses odometry (predict) with GPS (update) to estimate
    the robot's position and velocity.

    State vector: [x, y, vx, vy]
    """

    def __init__(
        self,
        dt: float = 0.1,
        process_noise: float = 0.1,
        measurement_noise: float = 2.0,
    ):
        self.dt = dt

        # State vector: [x, y, vx, vy]
        self.x = np.zeros((4, 1))

        # State covariance matrix
        self.P = np.eye(4) * 1.0

        # State transition matrix (constant velocity model)
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # Process noise covariance
        q = process_noise
        self.Q = np.array([
            [dt**4/4, 0, dt**3/2, 0],
            [0, dt**4/4, 0, dt**3/2],
            [dt**3/2, 0, dt**2, 0],
            [0, dt**3/2, 0, dt**2]
        ]) * q**2

        # Measurement matrix (GPS measures position only)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        # Measurement noise covariance (GPS noise)
        r = measurement_noise
        self.R = np.eye(2) * r**2

        # Identity matrix
        self.I = np.eye(4)

    def initialize(self, x=0.0, y=0.0, vx=0.0, vy=0.0, initial_uncertainty=1.0):
        self.x = np.array([[x], [y], [vx], [vy]])
        self.P = np.eye(4) * initial_uncertainty

    def predict(self):
        # State prediction: x = F @ x
        self.x = self.F @ self.x
        # Covariance prediction: P = F @ P @ F.T + Q
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z_x: float, z_y: float):
        z = np.array([[z_x], [z_y]])
        # Innovation
        y = z - self.H @ self.x
        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R
        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
        # State update
        self.x = self.x + K @ y
        # Covariance update
        self.P = (self.I - K @ self.H) @ self.P

    def get_state(self) -> Tuple[float, float, float, float]:
        return (
            float(self.x[0, 0]),
            float(self.x[1, 0]),
            float(self.x[2, 0]),
            float(self.x[3, 0]),
        )

    def get_covariance(self) -> np.ndarray:
        return self.P.copy()

    def get_position_uncertainty(self) -> Tuple[float, float]:
        return np.sqrt(self.P[0, 0]), np.sqrt(self.P[1, 1])
