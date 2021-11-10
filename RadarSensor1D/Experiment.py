import matplotlib.pyplot as plt
import numpy as np
from DataGenerationRadar1D import GenerateData, rangeAccuracy, velocityAccuracy
from KalmanFilter import KalmanFilter
from other_func import Q_discrete_white_noise

opt = {
        "initialDistance": 8,
        "stopTime": 1,
        "movementRange": 1,
        "frequency": 2,
        "SporadicError": 3,
        "velocity": 3
    }

timeAxis, distValues, velValues, truthDistValues, truthVelValues = GenerateData(type="Static", options=opt)

plt.figure()
plt.plot(timeAxis, distValues)
plt.plot(timeAxis, velValues)
plt.plot(timeAxis, truthDistValues)
plt.plot(timeAxis, truthVelValues)
plt.xlabel("time in s")
plt.legend(["Distance", "Velocity", "Truth distance", "Truth velocity"])
plt.title("Measurement Data of a 1D Radar Sensor")
plt.grid(True)
plt.show()

'''
Aufgabe:
1. Implementieren Sie ein Kalman-Filter, das die Messdaten als Eingangsdaten nimmt.
2. Testen Sie das Kalman-Filter mit verschiedener Objektbewegungsarten.
'''

# Hier Ihr Kalman-Filter initialisieren

# Messwerte
dt = .1
R_var = rangeAccuracy
Q_var = velocityAccuracy
x = np.array([distValues[0], velValues[0]]).T
#pred = [x] # Predictions array

# Kovarianz
P = np.diag([rangeAccuracy**2, velocityAccuracy**2])
R = np.array([[R_var]])
Q = Q_discrete_white_noise(dim=2, dt=dt, var=Q_var)

# Transition
F = np.array([[1, dt],
              [0,  1]])
H = np.array([[1., 0.]])

# Kalman Filter
kFilter = KalmanFilter(x, P, R, F, H, Q)
pred = []
for i in range(np.size(timeAxis)):
    # hier die Daten ins Kalman-Filter eingeben
    input = np.array([distValues[i], velValues[i]])
    output = kFilter.Step(input)
    pred.append(output)

# Hier das Ergebnis über die Zeit plotten.
plt.plot(pred)
# Um wie viel hat sich die Messgenauigkeit verbessert?
# Wie beeinflussen die Schätzung der Kovarianzmatrix Q und R die Genauigkeit
# Fügen Sie zufällige Messfehler mit der Parameter "SporadicError" hinzu, wie verhält sich das Kalman Filter?