import numpy as np


model_move = lambda est, dt: np.array([est[0] + est[1]*dt, 1])

class abFilter():
    def __init__(self, a, model):
        self.a = a
        self.epoch = 0
        self.estimate = None
        # Process model
        self.model = model
        
    def step(self, s, dt):
        
        if self.estimate is None:
            self.estimate = s
            return s
        
        else:
            # Predict
            s_pred = self.model(self.estimate,dt)
            
            # Update
            residual = s - s_pred
            self.estimate = self.estimate + self.a * residual
            return self.estimate


class KalmanFilter:
    # Initialisierung von Kalman Filter
    def __init__(self, s_hat, transition_model, H, Q, R):
        self.s_hat = s_hat
        self.P_hat = np.eye(len(s_hat)) * 100
        self.model = transition_model
        self.H = H # Measurement Function
        self.Q = Q # Process Noise
        self.R = R # Measurement Noise.
        pass


    def step(self,z):
        # Prediction
        s_hat_p = self.model @ self.s_hat
        P_hat_p = self.model @ self.P_hat @ self.model.T +  self.Q
        # Calculate Kalman Matrix
        K = P_hat_p @ self.H.T @ np.linalg.inv(self.H @ P_hat_p @ self.H.T + self.R)
        # Update covariance of estimation error
        self.P_hat = self.P_hat - K @ self.H @ self.P_hat
        # Improve estimate
        e_m_p = z - self.H @ s_hat_p
        self.s_hat = s_hat_p + K @ e_m_p
        
        return self.s_hat