import matplotlib.pyplot as plt
import numpy as np
from DataGenerationRadar1D import GenerateData
from KalmanFilter import KalmanFilter

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

# timeAxis = [0,1,2,3,4]
# distValues = [4000,4260,4550,4860,5110]
# velValues = [280,282,285,286,290]




# Hier Ihr Kalman-Filter initialisieren
kFilter = KalmanFilter(np.array([[distValues[0]],[velValues[0]]]), np.array([[0.4**2,0],[0,0.4**2]]), 0.01)
values = []

for i in range(np.size(timeAxis)):
    # hier die Daten ins Kalman-Filter eingeben
    output = kFilter.Step(np.array([[distValues[i]],[velValues[i]]]))
    values.append(output)


pDistValues = [values[i][0][0] for i in range(len(values))]
pVelValues = [values[i][1][0] for i in range(len(values))]
# Hier das Ergebnis über die Zeit plotten.
plt.figure()
# plt.plot(timeAxis, values)
plt.plot(timeAxis, pDistValues)
# plt.plot(timeAxis, pVelValues)
plt.plot(timeAxis, distValues)
# plt.plot(timeAxis, velValues)
# plt.legend(["Pred. Distance", "Pred. Velocity","Distance", "Velocity"])
plt.legend(["Pred. Distance","Distance"])
plt.grid(True)
plt.show()

# Um wie viel hat sich die Messgenauigkeit verbessert?
# Wie beeinflussen die Schätzung der Kovarianzmatrix Q und R die Genauigkeit
# Fügen Sie zufällige Messfehler mit der Parameter "SporadicError" hinzu, wie verhält sich das Kalman Filter?
print("finished")