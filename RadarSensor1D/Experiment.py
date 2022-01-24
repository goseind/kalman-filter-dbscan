import matplotlib.pyplot as plt
import numpy as np
from DataGenerationRadar1D import GenerateData, rangeAccuracy, velocityAccuracy
from KalmanFilter import KalmanFilter

opt = {
        "initialDistance": 8,
        "stopTime": 1,
        "movementRange": 1,
        "frequency": 2,
        "SporadicError": 5,
        "velocity": 3
    }

timeAxis, distValues, velValues = GenerateData(type="ConstantVelocity", options=opt)

plt.figure()
plt.plot(timeAxis, distValues)
plt.plot(timeAxis, velValues)
#plt.plot(timeAxis, truthDistValues)
#plt.plot(timeAxis, truthVelValues)
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

# Measurement Error
## Variance of a uniform distribution is given by (b-a)**2/12.
R = np.diag([rangeAccuracy**2, velocityAccuracy**2])/3
# todo: Add variance.
Q = np.diag([0,0,0])
# todo: add column for acceleration
s0 = np.array([distValues[0], velValues[0], 0])
#todo: Add acceleration.
transition_model = np.array([[1, 0.01, 0.01/2],
                             [0, 1, 0.01],
                             [0, 0, 0.01]])
# todo: adjust H for accomodating acceleration.
H =  np.array([[1., 0., 0.],
               [0., 1., 0.]])

KalmanFilter = KalmanFilter(s0, transition_model, H, Q, R)

Predictions = [s0]
for i in range(1,np.size(timeAxis)):
    s = np.array([distValues[i], velValues[i]])
    pred = KalmanFilter.step(s)
    Predictions.append(pred)
    
plt.plot(Predictions)


# Hier das Ergebnis über die Zeit plotten.

# Um wie viel hat sich die Messgenauigkeit verbessert?
# Wie beeinflussen die Schätzung der Kovarianzmatrix Q und R die Genauigkeit
# Fügen Sie zufällige Messfehler mit der Parameter "SporadicError" hinzu, wie verhält sich das Kalman Filter?
