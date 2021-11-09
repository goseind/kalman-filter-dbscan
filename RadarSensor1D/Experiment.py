import matplotlib.pyplot as plt
import numpy as np
from DataGenerationRadar1D import GenerateData, rangeAccuracy, velocityAccuracy
from KalmanFilter import KalmanFilter

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
x = np.array(distValues[0], velValues[0])

# Kovarianz
P = np.diag([rangeAccuracy**2, velocityAccuracy**2])

# Transition
dt = 0.1
F = np.array([1, dt],
             [0, 1])

# Kalman Filter
kFilter = KalmanFilter()

for i in range(np.size(timeAxis)):
    # hier die Daten ins Kalman-Filter eingeben
    # output = kFilter.Step(input)
    pass

# Hier das Ergebnis über die Zeit plotten.

# Um wie viel hat sich die Messgenauigkeit verbessert?
# Wie beeinflussen die Schätzung der Kovarianzmatrix Q und R die Genauigkeit
# Fügen Sie zufällige Messfehler mit der Parameter "SporadicError" hinzu, wie verhält sich das Kalman Filter?