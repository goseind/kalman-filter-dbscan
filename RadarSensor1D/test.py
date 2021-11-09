import numpy as np
import matplotlib.pyplot as plt

s_hat = [1, 2, 3, 4]

P_hat = np.eye(len(s_hat)) 

print(P_hat)

def volt(voltage, std):
    return voltage + (np.random.randn() * std)

rangeAccuracy = 2  # m
velocityAccuracy = 5  # m/s
P = np.diag([rangeAccuracy**2, velocityAccuracy**2])/3
print(P)