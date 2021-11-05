import numpy as np

# https://www.youtube.com/watch?v=CaCcOwJPytQ&list=PLX2gX-ftPVXU3oUFNATxGXY90AULiqnWT
class KalmanFilter:
    # Initialisierung von Kalman Filter
    def __init__(self, X, P, dt):
        self.X = X
        # self.X = np.array([[1],[1]]) #position, velocity
        # process error coveriance matrix
        self.P = P

        # delta t
        self.dt = dt
        self.A = np.array([[1,dt],[0,1]])
        self.B = np.array([[0.5*dt**2],[dt]])
        # control variable matrix
        self.u = 0
        # noise
        self.w = 0
        self.C = np.array([[1,0],[0,1]])
        #measurment noise
        self.z = 0

        self.Q = np.array([[0,0],[0,0]])

        # observation error matrix
        self.R = np.array([[1**2,0],[0,3**2]])
        pass

    # movement equation
    # x = x0 + v*t = 0.5*a*t^2

    # calc time
    # deltat = time for 1 cycle

    # accelaration could be the control variable

    def Step(self, MEA):
        # equations from: main calculations (from flowchart).png

        # predict new state
        # Xkp = A*Xk-1 + Buk + wk (X= State Matrix, u = Control Variable Matrix, w = Noise)
        # A and B to convert prediction to the next state
        # B * uk = new velocity based on accelarotion
        self.X = np.dot(self.A, self.X) + np.dot(self.B, self.u) + self.w

        # P = state covariance matrix(calculate error in the estimate)
        # Q = process noise covar. matrix (keeps P from getting too small or 0)
        # Pkp = APk-1 * AT + Qk
        self.P = np.dot(np.dot(self.A, self.P), np.transpose(self.A)) + self.Q


        # calculate H
        # used to change format to Kalman Gain
        H = np.identity(2)

        # R = measurement covariance matrix (error in measurement)
        # if R -> 0 then adjust primarly with measurment update
        # if R -> Large then adjust primarly with prediced state
        # if P -> 0 then measurement updates are mostly ignored
        # K = Pkp*H / (H*Pkp*HT + R)
        K = np.divide(self.P * np.transpose(H), (H * self.P * np.transpose(H) + self.R))
        K = np.nan_to_num(K)

        # Xk = Xkp + K*(Y-HXkp)
        # Y = C*Ym + Zk (C = What are we measuring)
        # only pos = [1,0], only vel = [0,1]
        # both = [[1,0],[0,1]]
        Y = np.dot(self.C, MEA) + self.z
        self.X = self.X + np.dot(K, (Y - np.dot(H, self.X)))

        return self.X
