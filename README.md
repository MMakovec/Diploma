# Diploma

Vodenje robota pololu 3pi+ po virtualnem okolju preko bluetooth povezave z A* algoritmom. Vključuje program napisan v ArduinoIDE, ki se naloži na robota, ter programe v pythonu, ki se izvajajo na računalniku. Regulator za robota je izveden direktno na robotu in preko računalnika. Za spreminjanje funkcionalnosti se uporabljata globalni spremenljivke robot in computer, s kombinacijo katerih določamo način sledenja lokacije robota ter kje se izvaja regulacija, kot je opisano v bt.py. Spremenljivki, ki določata funkcijonalnost se morata ujemati med 3pi.ino in bt.py.
