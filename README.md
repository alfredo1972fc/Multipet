# Multipet
Firmware para fileteado múltiple de botellas
- 11/11 3 fileteadores
. Al arrancar si tiene un filamento en el sensor, activa calentador, y al llegar a temperatura lo enciende para evitar parones por cortes de corriente
. Si ponemos un filamento en el sensor y esta en la temperatura adecuada, encenderá el motor

Debe tener siguientes librerias
./ AccelStepper.h 
./ thermistor.h
./ TimerOne.h
./ TimerThree.h
./ QuickPID.h
./ LiquidCrystal_I2C.h
./ LiquidMenu.h
./ EEPROM.h

Si usas platformio en el fichero de Platform.ini estan con el nombre del autor
