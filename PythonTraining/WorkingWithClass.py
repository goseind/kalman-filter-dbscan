'''
Dieser Skript dient zur Einführung in die Verwendung von Klassen in 
Python-Programmierungssprache.

Autor: Dr.-Ing. Wei Yap Tan
Datum: 06.09.2021
Lehrveranstaltung: Projektlabor Maschinelles Lernen

Fakultät der Informationstechnik
Hochschule Mannheim
'''

'''
Klassen können so definiert werden.
'''

class MyClass:
    def __init__(self, initValue): # Konstruktor mit Parameter
        self.varA = initValue  # Klassenvariable
        self.varB = 12

    def GetSum(self):
        return self.varA + self.varB

    def IncrementVar(self, ind):
        if(ind == 0):
            self.varA = self.varA + 1
        elif(ind == 1):
            self.varB = self.varB + 1
        else:
            print("Error: invalid index")

    def __add__(self, other): # Überladung von '+'-Operator
        varA = self.varA + other.varA
        return MyClass(varA)
        
    def __str__(self):
        return "{0},{1}".format(self.varA, self.varB)

'''
Verwendung von Klasse MyClass in diesem Skript:
'''

if __name__=="__main__": # lokale Main

    x = MyClass(2)
    x.IncrementVar(1)
    y = MyClass(4)
    z = x + y

    print("x: ", x)
    print("y: ", y)
    print("z: ", z)

'''
Verwendung von Klasse MyClass in anderem Skript
siehe UsingExternalClass.py
'''

''' 
Aufgabe: Definieren Sie eine Funktion mit dem Name
'DecementVar', die das entsprechende Variable der Klasse
MyClass dekrementiert.

Für ind == 0 wird die Variable varA dekrementiert
Für ind == 1 wird die Variable varB dekrementiert
Für alle andere Werte von ind wird ein Fehler ausgegeben.
'''