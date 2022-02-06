# Projektlabor Maschinelles Lernen (PML)

|Gruppenmitglied|Matrikelnummer|
|---|---|
|Christian Singer|2161064|
|Domenic Gosein|2160647|
|Lukas Burger|2150580|
|Maximilian Kürschner|2160331|

Dozent: Dr.-Ing. Wei Yap Tan  

Fakultät der Informationstechnik  

Hochschule Mannheim  
Wintersemester 2021/22  

## Aufgaben

Teilaufgabe 1:

- [x] Implementierung des Kalman-Filters für einen 1D-Radarsensor
- [x] Stabilität des Kalman-Filters für verschiedene Bewegungsarten erreicht
- [x] Systematische Anpassung der Q und R-Matrix für verschiedene Bewegungsarten
- [ ] Verbesserung der Messgenauigkeit des 1D-Radarsensors
- [ ] Bonus: Strategie zur Erkennung der Bewegungsarten, und somit eine Echtzeitanpassung des Kalman-Filters

Teilaufgabe 2:

- [x] Implementierung des DBScan-Verfahren zur Clusterbildung mit Daten aus dem 3D-Radarsensor
- [x] Erfolgreiche Clustering mehrerer Objekten + Erkennung von Ausreißer
- [x] Stabile Clustering-Ergebnis für mehrere Objekte mit unterschiedlicher Bewegungspfaden
- [x] Anpassung des Kalman-Filters für 3D
- [x] Sinnvolle Verwendung der Clustering-Ergebnisse als Eingangsdaten des Kalman-Filters
- [ ] Verbesserung der Messgenauigkeit + Detektion von Ausreißer

## Implementierung

* [Kalman Filter 1D](/RadarSensor1D/KalmanFilter.py)
* [DBSCAN 3D](/RadarSensor3D/DBScan.py)
* [Kalman Filter 3D](/RadarSensor3D/KalmanFilter.py)

## Dokumentation

* [Bericht als Jupyter Notebook](bericht.ipynb)
* [Bericht als PDF](bericht.pdf)
* [Präsentation online](https://docs.google.com/presentation/d/1tMrf8yo_mv9Uo4Sv87vI6JIYx5SxoOuYC4IeCOa-haQ/edit?usp=sharing)
* [Präsentation als PDF]()