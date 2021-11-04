import matplotlib.pyplot as plt
import numpy
from DataGenerationRadar1D import GenerateData
from KalmanFilter1D import KalmanFilter

opt = {
        "initialDistance": 8,
        "stopTime": 1,
        "movementRange": 1,
        "frequency": 2,
        "SporadicError": 3
    }

timeAxis, distValues, velValues, truthDistValues, truthVelValues = GenerateData(type="Sinus", options=opt)

# plt.figure()
# plt.plot(timeAxis, distValues)
# plt.plot(timeAxis, velValues)
# plt.plot(timeAxis, truthDistValues)
# plt.plot(timeAxis, truthVelValues)
# plt.xlabel("time in s")
# plt.legend(["Distance", "Velocity", "Truth distance", "Truth velocity"])
# plt.title("Measurement Data of a 1D Radar Sensor")
# plt.grid(True)
# plt.show()

'''
Aufgabe:
1. Implementieren Sie ein Kalman-Filter, das die Messdaten als Eingangsdaten nimmt.
2. Testen Sie das Kalman-Filter mit verschiedener Objektbewegungsarten.
'''

# Hier Ihr Kalman-Filter initialisieren
kFilter = KalmanFilter(2, 68, 4)
timeAxis = [1,2,3,4]
tempValues = [75,71,70,74]
trueValue = 72
values = []

for i in range(numpy.size(timeAxis)):
    # hier die Daten ins Kalman-Filter eingeben
    output = kFilter.Step(tempValues[i])
    values.append(output)

# Hier das Ergebnis über die Zeit plotten.
plt.figure()
plt.plot(timeAxis, values)
plt.plot(timeAxis, tempValues)
plt.plot(timeAxis, [trueValue, trueValue, trueValue, trueValue])
plt.grid(True)
plt.show()

# Um wie viel hat sich die Messgenauigkeit verbessert?
# Wie beeinflussen die Schätzung der Kovarianzmatrix Q und R die Genauigkeit
# Fügen Sie zufällige Messfehler mit der Parameter "SporadicError" hinzu, wie verhält sich das Kalman Filter?