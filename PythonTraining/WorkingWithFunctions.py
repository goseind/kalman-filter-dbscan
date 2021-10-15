'''
In diesem Skript lernen Sie den Umgang mit Funktionen in Python

Autor: Dr.-Ing. Wei Yap Tan
Datum: 06.09.2021
Lehrveranstaltung: Projektlabor Maschinelles Lernen

Fakultät der Informationstechnik
Hochschule Mannheim
'''

'''
Eine Funktion wird in Python so definiert:
'''

def PrintMessage(Message):
    print(Message)

# Die Funktion kann in der Datei aufgeruft werden
PrintMessage("Beispiel Text")

'''
Die Rückgabewerten wird mit der Return-Anweisung zurückgegeben
'''

def GetTextAndValue():
    return "Text", 2 # Ausgabe von mehrerer Werten werden mit dem Komma getrennt.


txt, val = GetTextAndValue()
print("txt:", txt)
print('val: ', val)

'''
Aufgabe: Definieren Sie eine Funktion die die Summe aller Elementen im Array zurückgibt
'''

def GetSumOfArray(array):
    '''
    To-Do
    '''
    

'''
Testen Sie Ihre Funktion
'''
x = [1, 3, 5, 7, 9]

summe = GetSumOfArray(x)
print("summe: ", summe)