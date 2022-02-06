import numpy as np

class KalmanFilter:
    # Initialisierung von Kalman Filter
    def __init__(self, s_hat, transition_model, H, Q, R):
        self.s_hat = s_hat
        self.P_hat = np.eye(len(s_hat)) * 100
        self.model = transition_model
        self.H = H # Measurement Function
        self.Q = Q # Process Noise
        self.R = R # Measurement Noise.

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

def compute_mse(pred_val, truth_val, val):
    diff_sensor_pred = pred_val - truth_val
    diff_sensor_data = val - truth_val

    mse_sens_pred = np.sum(diff_sensor_pred ** 2) / np.prod(diff_sensor_pred.shape)

    mse_sens_data = np.sum(diff_sensor_data ** 2) / np.prod(diff_sensor_data.shape)

    return mse_sens_pred, mse_sens_data, mse_sens_data - mse_sens_pred