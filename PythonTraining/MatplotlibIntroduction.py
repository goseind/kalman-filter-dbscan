'''
Dieser Skript dient zur Einführung in die Verwendung von Matplotlib in 
Python-Programmierungssprache.

Autor: Dr.-Ing. Wei Yap Tan
Datum: 06.09.2021
Lehrveranstaltung: Projektlabor Maschinelles Lernen

Fakultät der Informationstechnik
Hochschule Mannheim
'''

import numpy as np
import matplotlib.pyplot as plt

A = np.random.randn(100) # 100 normalverteilte Zufahlszahlen

plt.figure()
plt.plot(A)
plt.xlabel("index")
plt.ylabel("value")
plt.title("Example Plot")
plt.show()

'''
Mehr Beispiele unter
https://matplotlib.org/stable/gallery/index.html
'''