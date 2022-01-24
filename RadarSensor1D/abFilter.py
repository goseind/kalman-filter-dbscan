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