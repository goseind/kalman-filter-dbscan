'''
Dieser Skript dient zur Einführung in die Python-Programmierungssprache.

Autor: Dr.-Ing. Wei Yap Tan
Datum: 06.09.2021
Lehrveranstaltung: Projektlabor Maschinelles Lernen

Fakultät der Informationstechnik
Hochschule Mannheim
'''

'''
Python-Module können mit der 'import'-Anweisung inkludiert werden.
'''
import os

'''
Kommentare können so aussehen.
'''
# Diese is ein Inline-Kommentar in Python

'''
Arbeiten mit Variablen:

Variablen können jede Zeit definiert werden.
Ein Variable wird nich deklariert, sondern direkt mit einem Wert definiert wird.
'''

# Beispiel:

x = 5  # integer
y = "Text"  # string
z = 'Text'  # string
b = 4.3  # float
arrayX = [3, 2]  # integer array
matrixY = [[3,2],[4,3],[5,1]] # integer matrix 3 x 2

summe = x + b # 5 + 4.3 = 9.3
print("summe: ", summe) # Ausgabe 

'''
Anhängen von Werte ins Array
'''
arrayX.append(1)
print("arrayX: ", arrayX)

'''
Werte in Matrix verändern
'''
matrixY[0][1] = 0
print("matrixY: ", matrixY)

'''
string verlängern
'''
y = y + ", Hallo"
print("y: ", y)

'''
For und While-Schleife

Python arbeitet mit dem Einzug.
Alle Code-Zeile mit dem gleichen Einzug unterhalb gehören zusammen unter der For-Anweisung

'''

counter = 0 
print("Ausgabe der for-Schleife:")
for i in range(0,10): # For-Schleife zählt von 0 bis 10, die Zahl 10 ist aber exkludiert.
    counter = counter + i
    print(counter)

print("Ausgabe der while-Schleife:")
j = 0
while (j <= 100):
    if(j % 10 == 0):
        print(j)
    
    j = j + 1

