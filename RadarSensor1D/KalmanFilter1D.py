class KalmanFilter:
    # Initialisierung von Kalman Filter
    def __init__(self, Eest, EST, Emea):
        self.Eest = Eest
        self.EST = EST
        self.Emea = Emea

        self.KGarray = []
        pass

    # Diese Funktion nimmt die Messwerten und gibt 
    # das Ergebnis des Kalman Filters zurück
    #
    # Bitte hier geeignete Eingabe- und Rückgabeparametern ergänzen
    def Step(self, MEA):
        # equations from: main calculations (from flowchart).png
        # KG = Eest / (Eest + Emea)
        KG = self.Eest / (self.Eest + self.Emea)
        self.KGarray.append(KG)
        # ESTt = ESTt-1 + KG(MEA - ESTt-1)
        self.EST = self.EST + KG*(MEA - self.EST)
        # Eest(t) = (1-KG)Eest(t-1)
        self.Eest = (1-KG)*self.Eest

        return self.EST
