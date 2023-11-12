# Multipet
Firmware para fileteado múltiple de botellas
- 11/11 3 fileteadores
. Al arrancar si tiene un filamento en el sensor, activa calentador, y al llegar a temperatura lo enciende para evitar parones por cortes de corriente
. Si ponemos un filamento en el sensor y esta en la temperatura adecuada, encenderá el motor

Debe tener siguientes librerias
#include <AccelStepper.h>
#include <thermistor.h>
#include <TimerOne.h>
#include <TimerThree.h>
#include <QuickPID.h>
#include <LiquidCrystal_I2C.h> // Librería para controlar el LCD
#include <LiquidMenu.h>
#include <EEPROM.h>

Si usas platformio en el fichero de configuracion estan con el nombre del autor
