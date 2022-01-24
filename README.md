# Projektlabor Maschinelles Lernen (PML)

Dozent: Dr.-Ing. Wei Yap Tan\
Fakultät der Informationstechnik\
Hochschule Mannheim\
WiSe 2021/22

# Aufgaben

Teilaufgabe 1:

- [ ] Implementierung des Kalman-Filters für einen 1D-Radarsensor
- [ ] Stabilität des Kalman-Filters für verschiedene Bewegungsarten erreicht
- [ ] Systematische Anpassung der Q und R-Matrix für verschiedene Bewegungsarten
- [ ] Verbesserung der Messgenauigkeit des 1D-Radarsensors
- [ ] Bonus: Strategie zur Erkennung der Bewegungsarten, und somit eine Echtzeitanpassung des Kalman-Filters

Teilaufgabe 2:

- [ ] Implementierung des DBScan-Verfahren zur Clusterbildung mit Daten aus dem 3D-Radarsensor
- [ ] Erfolgreiche Clustering mehrerer Objekten + Erkennung von Ausreißer
- [ ] Stabile Clustering-Ergebnis für mehrere Objekte mit unterschiedlicher Bewegungspfaden
- [ ] Anpassung des Kalman-Filters für 3D
- [ ] Sinnvolle Verwendung der Clustering-Ergebnisse als Eingangsdaten des Kalman-Filters
- [ ] Verbesserung der Messgenauigkeit + Detektion von Ausreißer

## Gruppe und Mitglieder

**Gruppe 7:**
* Christian Singer
* Domenic Gosein
* Lukas Burger
* Maximilian Kürschner

## Link zu den Implementierungen

* Kalman Filter 1D: [RadarSensor1D](/RadarSensor1D/KalmanFilter.py)
* DBSCAN 3D: [RadarSensor3D](/RadarSensor3D/DBScan.py)
* Kalman Filter 3D: [RadarSensor1D](/RadarSensor3D/KalmanFilter.py)

## Dokumentation

Die Dokumentation und Berichtersattung erfolgt in folgendem [Jupyter Notebook](bericht.ipynb) & [Link zur Präsentation](https://docs.google.com/presentation/d/1tMrf8yo_mv9Uo4Sv87vI6JIYx5SxoOuYC4IeCOa-haQ/edit?usp=sharing)

## Software Voraussetzungen

* python
* scipy
* numpy
* sklearn
* matplotlib