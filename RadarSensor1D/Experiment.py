import matplotlib.pyplot as plt
import numpy as np
from DataGenerationRadar1D import GenerateData, rangeAccuracy, velocityAccuracy # Import variables too
from KalmanFilter import KalmanFilter
from WhiteNoise import Q_discrete_white_noise # Import noise function

opt = {
        "initialDistance": 8,
        "stopTime": 1,
        "movementRange": 1,
        "frequency": 2,
        "SporadicError": 3,
        "velocity": 3 # Add velocity
    }

timeAxis, distValues, velValues, truthDistValues, truthVelValues = GenerateData(type="Static", options=opt)

'''
Aufgabe:
1. Implementieren Sie ein Kalman-Filter, das die Messdaten als Eingangsdaten nimmt.
2. Testen Sie das Kalman-Filter mit verschiedener Objektbewegungsarten.
'''

# Kalman Filter Initialization

## Parameters and values
dt = .1 #Time step
R_var = rangeAccuracy
Q_var = velocityAccuracy
x = np.array([[distValues[0], velValues[0]]]).T
P = np.diag([rangeAccuracy**2, velocityAccuracy**2])
F = np.array([[1, dt],
              [0,  1]])
H = np.array([[1., 0.]])
R = np.array([[R_var]])
Q = Q_discrete_white_noise(dim=2, dt=dt, var=Q_var)
pred = [] # Array for predictions
cov = []

## Kalman Filter Instance
kFilter = KalmanFilter(x, P, F, H, R, Q)

## Data loop to go through all dist/vel values
for i in range(np.size(timeAxis)):
    input = distValues[i] # Vel Value??
    output = kFilter.Step(input)
    pred.append(output)

## Plot predictions
plt.figure()
plt.plot(timeAxis, distValues)
plt.plot(timeAxis, velValues)
plt.plot(timeAxis, truthDistValues)
plt.plot(timeAxis, truthVelValues)
plt.plot(timeAxis, np.squeeze(pred))
plt.xlabel("time in s")
plt.legend(["Distance", "Velocity", "Truth distance", "Truth velocity", "Prediction"])
plt.title("Measurement Data of a 1D Radar Sensor")
plt.grid(True)
plt.show()

'''
Questions:
* Um wie viel hat sich die Messgenauigkeit verbessert?
* Wie beeinflussen die Schätzung der Kovarianzmatrix Q und R die Genauigkeit
* Fügen Sie zufällige Messfehler mit der Parameter "SporadicError" hinzu, wie verhält sich das Kalman Filter?
'''