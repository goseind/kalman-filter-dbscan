from matplotlib.pyplot import xcorr
import numpy as np
from scipy.linalg import inv
from other_func import Q_discrete_white_noise # Import noise function

## Kalman Filter Class

class KalmanFilter:

    # Class Initialization
    def __init__(self, x, P, F, H, R, Q):
        self.x = x
        self.P = P
        self.H = H
        self.F = F
        self.R = R
        self.Q = Q

    # Predict and update function of the Kalman Filter
    def Step(self, z):
        # Predict values
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

        # Update values
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ inv(S)
        y = z - self.H @ self.x
        self.x += K @ y
        self.P = self.P - K @ self.H @ self.P

        # Return value
        return self.x #, self.P