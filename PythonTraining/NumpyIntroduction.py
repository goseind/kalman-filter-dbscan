'''
Dieser Skript dient zur Einführung in die Verwendung von Numpy in 
Python-Programmierungssprache.

Autor: Dr.-Ing. Wei Yap Tan
Datum: 06.09.2021
Lehrveranstaltung: Projektlabor Maschinelles Lernen

Fakultät der Informationstechnik
Hochschule Mannheim
'''

import numpy as np # importiert numpy mit dem Name 'np'

'''
Arbeiten mit array
'''

# Einfache Erzeugung vom Array

x = np.array([3,2,1])
print('x: ', x)
print('shape of x: ', np.shape(x)) 
# Achtung! die Dimension des Array is (3,) statt (3,1)
# 1D-Array wird nicht als liegender oder stehender Vektor unterschieden

# Umwandlung von 1D-Array -> Matrix
y = np.reshape(x, (3,1))
print('y: ', y)
print('shape of y: ', np.shape(y)) 

z = y.transpose()
print('z: ', z)
print('shape of z: ', np.shape(z)) 

# Null-Matrix/One-Matrix erzeugen
a = np.zeros((3,4))
print('a: ', a)
print('shape of a: ', np.shape(a)) 

b = np.ones((2,3))
print('b: ', b)
print('shape of b: ', np.shape(b)) 

# Elementwise Operation
A = np.array([[1,1],[0,1]])
B = np.array([[2,0],[3,4]])

C = A * B # Elementweise multipliziert

D = np.dot(A, B) # Matrix Produkt
# Alternativ: D = A.dot(B)

print("C: ", C)
print("D: ", D)

# Array mit n-Elemente zwischen zwei Grenzwerten
E = np.linspace(0,1,5, endpoint=False) # Stop-Wert von '1' wird exkludiert
print("E: ", E)

F = np.linspace(0,1,5, endpoint=True) # Stop-Wert von '1' wird inkludiert
print("F: ", F)

# Indexing, Slicing
# Zugriff auf Elementen

print(E[1:3]) # Achtung! Index 3 ist nicht inkludiert.
print(A[:,0]) # Erste Spalte
print(A[0,:]) # Erste Zeile

'''
Weitere Operationen finden Sie auf der Doku-Seite von Numpy
https://numpy.org/doc/stable/index.html

'''