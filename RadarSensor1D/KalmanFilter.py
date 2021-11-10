from matplotlib.pyplot import xcorr
import numpy as np
from scipy.linalg import inv
from other_func import Q_discrete_white_noise

class KalmanFilter:
    # Initialisierung von Kalman Filter
    def __init__(self, x, P, R, F, H, Q):
        self.x = x
        self.P = P
        self.R = R
        self.F = F
        self.H = H
        self.Q = Q

    # Diese Funktion nimmt die Messwerten und gibt 
    # das Ergebnis des Kalman Filters zurück
    
    # Bitte hier geeignete Eingabe- und Rückgabeparametern ergänzen
    def Step(self, z):
        # Predict
        x = self.F @ self.x
        P = self.F @ self.P @ self.F.T + self.Q

        # Update
        S = self.H @ self.P @ self.H.T + self.R
        K = P @ self.H.T @ inv(S)
        y = z - self.H @ x
        x += K @ y
        P = P - K @ self.H @ P

        # Return value
        return x