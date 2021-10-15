'''
Dieser Skript dient zur Einführung in die Verwendung von Klassen in 
Python-Programmierungssprache.

Autor: Dr.-Ing. Wei Yap Tan
Datum: 06.09.2021
Lehrveranstaltung: Projektlabor Maschinelles Lernen

Fakultät der Informationstechnik
Hochschule Mannheim
'''

from WorkingWithClass import MyClass

x = MyClass(2)
x.IncrementVar(1)
y = MyClass(4)
z = x + y

print("x: ", x)
print("y: ", y)
print("z: ", z)
