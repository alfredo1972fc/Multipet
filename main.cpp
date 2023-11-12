#include <AccelStepper.h>
#include <thermistor.h>
#include <TimerOne.h>
#include <TimerThree.h>
#include <QuickPID.h>
#include <LiquidCrystal_I2C.h> // Librería para controlar el LCD
#include <LiquidMenu.h>
#include <EEPROM.h>
#include "menudos.h"

//  ************************** REVISAR, HACER  **************
//  hacer funcion sonido por una salida digital
//  Poner un poton para contabilizar botellas
//  si motor intenta moverse por debajo de temperatura, se para

int direccion=0;
long botella=10;
int TempBotella=0;
long leerBotella();
void grabarBotella();
void sacarBotellas();
//*************************** PARTE PANTALLA

void updateLCD();
static unsigned long previousMillis = 0; //variable para mostrar cada segundo
const unsigned long interval = 3000;

// ********************* PINS DE ENTRADA SENSORES
#define PIN_INPUT 14    // PIN SENSOR TEMPERATURA 1
#define PIN_INPUT2 13   // PIN SENSOR TEMPERATURA 2
#define PIN_INPUT3 15   // PIN SENSOR TEMPERATURA 3

// ********************* PINS DE SALIDA CALENTADORES
#define PIN_OUTPUT 9    // SALIDA CALENTADOR 1
#define PIN_OUTPUT2 10  // SALIDA CALENTADOR 2
#define PIN_OUTPUT3 8   // SALIDA CALENTADOR 3

// *********************  DEFINICIONES TEMPERATURA ****************
const uint32_t sampleTimeUs = 100000; // 100ms
volatile bool computeNow = false;
thermistor therm1(PIN_INPUT, 1); 
thermistor therm2(PIN_INPUT2, 1);
thermistor therm3(PIN_INPUT3, 1);

// Define variables del control del PID
float Setpoint=0, Setpoint2=0, Setpoint3=0, Output=0, Output2=0, Output3=0;
float Input=0, Input2=0, Input3=0;
float Kp = 26.5, Ki = 2.84, Kd = 61.8;

// DEFINIENDO OBJETOS CONTROL PID
QuickPID myPID(&Input, &Output, &Setpoint);
QuickPID myPID2(&Input2, &Output2, &Setpoint2);
QuickPID myPID3(&Input3, &Output3, &Setpoint3);
void runPid();

// ************************ PARTE DE MOTORES ********************
// Definir los pines y configuraciones de los motores
volatile bool mover=false;
#define STEP_PIN_X 54
#define DIR_PIN_X 55
#define ENABLE_PIN_X 38

#define STEP_PIN_Y 60
#define DIR_PIN_Y 61
#define ENABLE_PIN_Y 56

#define STEP_PIN_Z 46
#define DIR_PIN_Z 48
#define ENABLE_PIN_Z 62

AccelStepper stepperX(AccelStepper::DRIVER, STEP_PIN_X, DIR_PIN_X);
AccelStepper stepperY(AccelStepper::DRIVER, STEP_PIN_Y, DIR_PIN_Y);
AccelStepper stepperZ(AccelStepper::DRIVER, STEP_PIN_Z, DIR_PIN_Z);
int VelX=550, VelY=550, VelZ=550; // velocidades diferentes motores

void RunMotor();

// ************************* PARTE BOTONES-SENORES ENTRADA
// Definir los pines de los sensores
#define SENSOR_PIN1 3  // Pin del Sensor filamento 1
#define SENSOR_PIN2 2  // Pin del Sensor filamento 2
#define SENSOR_PIN3 14 // Pin del Sensor filamento 3
#define BotonStart1 18 // bonton arranque motor 1
#define BotonStart2 15 // Boton arranque motor 2
#define BotonStart3 19 // bonton arranque motor 3

// Definir botones de arranque i paro de cada motor
bool StartMotor1 = false;
bool StartMotor2 = false;
bool StartMotor3 = false;

void VerificaBotones();

void setup()
{
  Serial.begin(9600);
  //grabarBotella(valorInicial);
  long valorLeido = leerBotella();
  botella =valorLeido;
  //************ para ir contando botellas que vamos teniendo
  botella = botella +10;
  //




  // ************************ CONFIGURANDO LCD  ******
  lcd.begin(LCD_COLS, LCD_ROWS);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("fileteador 1.1");

  //************************** CONFIGURANDO COMO SALIDA PINES SALIDA PID ********
  pinMode(PIN_OUTPUT, OUTPUT);
  pinMode(PIN_OUTPUT2, OUTPUT);
  pinMode(PIN_OUTPUT3, OUTPUT);

  Timer1.initialize(sampleTimeUs);        // initialize timer1, and set the time interval
  Timer1.attachInterrupt(runPid); // attaches runPid() as a timer overflow interrupt
  Timer3.initialize(sampleTimeUs/100); 
  Timer3.attachInterrupt(RunMotor);
  

   

  //  APLICAR VALORES INICIALES
  myPID.SetTunings(Kp, Ki, Kd);
  myPID2.SetTunings(Kp, Ki, Kd);
  myPID3.SetTunings(Kp, Ki, Kd);

  // ENCENDER PID SALIDA CALENTADORES
  myPID.SetMode(myPID.Control::automatic);
  myPID2.SetMode(myPID2.Control::automatic);
  myPID3.SetMode(myPID3.Control::automatic);

  // ********************* CONFIGURANDO MOTORES **************
  stepperX.setMaxSpeed(10000);
  stepperX.setSpeed(5000);
  stepperX.setAcceleration(1000);
  stepperX.setEnablePin(ENABLE_PIN_X);          // Asignar el pin de habilitación para el motor X
  stepperX.setPinsInverted(false, false, true); // Configurar la inversión de pines según sea necesario

  stepperY.setMaxSpeed(10000);
  stepperY.setSpeed(5000);
  stepperY.setAcceleration(1000);
  stepperY.setEnablePin(ENABLE_PIN_Y);          // Asignar el pin de habilitación para el motor Y
  stepperY.setPinsInverted(false, false, true); // Configurar la inversión de pines según sea necesario

  stepperZ.setMaxSpeed(5000);
  stepperZ.setSpeed(5000);
  stepperZ.setAcceleration(1000);
  stepperZ.setEnablePin(ENABLE_PIN_Z);          // Asignar el pin de habilitación para el motor Y
  stepperZ.setPinsInverted(false, false, true); // Configurar la inversión de pines
  
  // Habilitar los PINES ENABLE DE LOS motores por defecto
  digitalWrite(ENABLE_PIN_X, LOW);
  digitalWrite(ENABLE_PIN_Y, LOW);
  digitalWrite(ENABLE_PIN_Z, LOW);

  // CONFIGURAR PINES DE ENTRADA
  pinMode(SENSOR_PIN1, INPUT_PULLUP); // Configurar el pin del sensor "Cable1" como entrada con resistencia pull-up
  pinMode(SENSOR_PIN2, INPUT_PULLUP); // Configurar el pin del sensor "Cable2" como entrada con resistencia pull-up
  pinMode(SENSOR_PIN3, INPUT_PULLUP);           
  pinMode(BotonStart1, INPUT_PULLUP); // boton para arrancar motor 1
  pinMode(BotonStart2, INPUT_PULLUP); // boton para arrancar motor 2
  pinMode(BotonStart3, INPUT_PULLUP);

  // variables de temperatura
  // ******* se activa temperatura si el sensor de filamento tiene dentro
  Setpoint = 0;  // estrusor 1
  Setpoint2 = 0;  // estrusor 1
  Setpoint3 = 0;  // estrusor 1
  if (digitalRead(SENSOR_PIN1) == LOW){Setpoint=220;}
  if (digitalRead(SENSOR_PIN2) == HIGH){Setpoint2=220;}
  if (digitalRead(SENSOR_PIN3) == LOW){Setpoint3=220;}





// ********************************** seccion menu ******************
  // DEFINIR PINES DE ENCODER MENU
  pinMode(sw,INPUT_PULLUP);
  pinMode(outputA,INPUT_PULLUP);
  pinMode(outputB,INPUT_PULLUP);

//añadir lineas a pantalla 0
//lineas de pantalla eleccion motores
  pantalla0.add_line(linea1_pant0);
  pantalla0.add_line(linea2_pant0);
  pantalla0.add_line(linea3_pant0);
  pantalla0.add_line(linea4_pant0);

//añadir lineas a pantalla 1
  pantalla1.add_line(linea1_pant1);
  pantalla1.add_line(linea2_pant1);
  pantalla1.add_line(linea3_pant1);
  pantalla1.add_line(linea4_pant1);
  pantalla1.add_line(linea5_pant1);

//añadir lineas a pantalla 2
  pantalla2.add_line(linea1_pant2);
  pantalla2.add_line(linea2_pant2);
  pantalla2.add_line(linea3_pant2);
  pantalla2.add_line(linea4_pant2);
  pantalla2.add_line(linea5_pant2);

//añadir lineas a pantalla 3
  pantalla3.add_line(linea1_pant3);
  pantalla3.add_line(linea2_pant3);
  pantalla3.add_line(linea3_pant3);
  pantalla3.add_line(linea4_pant3);
  pantalla3.add_line(linea5_pant3);

//añadir lineas a pantalla 4
  pantalla4.add_line(linea1_pant4);
  pantalla4.add_line(linea2_pant4);
  pantalla4.add_line(linea3_pant4);
  pantalla4.add_line(linea4_pant4);
  pantalla4.add_line(linea5_pant4);  

//añadir lineas a pantalla 5
  pantalla5.add_line(linea1_pant5);
  pantalla5.add_line(linea2_pant5);
  pantalla5.add_line(linea3_pant5);
  pantalla5.add_line(linea4_pant5);
  pantalla5.add_line(linea5_pant5);

  
  //definir posicion del selector lineas de patalla 0
  linea1_pant0.set_focusPosition(Position::LEFT); 
  linea2_pant0.set_focusPosition(Position::LEFT); 
  linea3_pant0.set_focusPosition(Position::LEFT); 
  linea4_pant0.set_focusPosition(Position::LEFT);  

  //definir posicion del selector lineas de patalla 1
  linea1_pant1.set_focusPosition(Position::LEFT); 
  linea2_pant1.set_focusPosition(Position::LEFT); 
  linea3_pant1.set_focusPosition(Position::LEFT); 
  linea4_pant1.set_focusPosition(Position::LEFT); 
  linea5_pant1.set_focusPosition(Position::LEFT); 

  //definir posicion del selector lineas de patalla 2
  linea1_pant2.set_focusPosition(Position::LEFT); 
  linea2_pant2.set_focusPosition(Position::LEFT); 
  linea3_pant2.set_focusPosition(Position::LEFT); 
  linea4_pant2.set_focusPosition(Position::LEFT); 
  linea5_pant2.set_focusPosition(Position::LEFT); 

  //definir posicion del selector lineas de patalla 3
  linea1_pant3.set_focusPosition(Position::LEFT); 
  linea2_pant3.set_focusPosition(Position::LEFT); 
  linea3_pant3.set_focusPosition(Position::LEFT); 
  linea4_pant3.set_focusPosition(Position::LEFT); 
  linea5_pant3.set_focusPosition(Position::LEFT);

  //definir posicion del selector lineas de patalla 4
  linea1_pant4.set_focusPosition(Position::LEFT); 
  linea2_pant4.set_focusPosition(Position::LEFT); 
  linea3_pant4.set_focusPosition(Position::LEFT); 
  linea4_pant4.set_focusPosition(Position::LEFT); 
  linea5_pant4.set_focusPosition(Position::LEFT);

  //definir posicion del selector lineas de patalla 5
  linea1_pant5.set_focusPosition(Position::LEFT); 
  linea2_pant5.set_focusPosition(Position::LEFT); 
  linea3_pant5.set_focusPosition(Position::LEFT); 
  linea4_pant5.set_focusPosition(Position::LEFT); 
  linea5_pant5.set_focusPosition(Position::LEFT);  


 
  linea1_pant0.attach_function(1, motor1); 
  linea2_pant0.attach_function(1, motor2); 
  linea3_pant0.attach_function(1, motor3); 
  linea4_pant0.attach_function(1, fn_todos); 

  linea1_pant1.attach_function(1, fn_ir_a_pantalla5); 
  linea2_pant1.attach_function(1, fn_ir_a_pantalla4); 
  linea3_pant1.attach_function(1, fn_ir_a_pantalla2); 
  linea4_pant1.attach_function(1, fn_vacio); 
  linea5_pant1.attach_function(1, fn_atras);

  linea1_pant2.attach_function(1, fn_preset1); 
  linea2_pant2.attach_function(1, fn_preset2); 
  linea3_pant2.attach_function(1, fn_preset3); 
  linea4_pant2.attach_function(1, fn_vacio); 
  linea5_pant2.attach_function(1, fn_atras);

  linea1_pant3.attach_function(1, fn_vacio); 
  linea2_pant3.attach_function(1, fn_vacio); 
  linea3_pant3.attach_function(1, fn_vacio); 
  linea4_pant3.attach_function(1, fn_vacio); 
  linea5_pant3.attach_function(1, fn_atras);
  
  linea1_pant4.attach_function(1, fn_on); 
  linea2_pant4.attach_function(1, fn_off); 
  linea3_pant4.attach_function(1, fn_mas); 
  linea4_pant4.attach_function(1, fn_menos); 
  linea5_pant4.attach_function(1, fn_atras);

  linea1_pant5.attach_function(1, tmp_on); 
  linea2_pant5.attach_function(1, tmp_off); 
  linea3_pant5.attach_function(1, tmp_menos); 
  linea4_pant5.attach_function(1, tmp_mas); 
  linea5_pant5.attach_function(1, fn_atras);


  menu.add_screen(pantalla0);
  menu.add_screen(pantalla1);
  menu.add_screen(pantalla2);
  menu.add_screen(pantalla3);
  menu.add_screen(pantalla4);
  menu.add_screen(pantalla5);
  
  pantalla0.set_displayLineCount(1);
  pantalla1.set_displayLineCount(1);
  pantalla2.set_displayLineCount(1);
  pantalla3.set_displayLineCount(1);
  pantalla4.set_displayLineCount(1);
  pantalla5.set_displayLineCount(1);

  menu.init();
  menu.set_focusedLine(0);
}

void loop()
{
  // **************** CONTROL PID *********
  if (computeNow)
  {
    Input = therm1.analog2temp();  // LEER TEMPERATURA
    Input2 = therm2.analog2temp(); // LEER TEMPERATURA
    Input3 = therm3.analog2temp(); // LEER TEMPERATURA
    myPID.Compute();                
    myPID2.Compute();
    myPID3.Compute();
    analogWrite(PIN_OUTPUT, Output);
    analogWrite(PIN_OUTPUT2, Output2);
    analogWrite(PIN_OUTPUT3, Output3);
    computeNow = false;
  }

  // *****************************  se verifican botones cada vez **********
  VerificaBotones();
  Menudisplay();

  // FUNCION LLAMADA CADA SEGUNDO
  
  unsigned long currentMillis = millis();
  // Verificar si ha transcurrido un segundo
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis; // Actualizar el tiempo anterior para el siguiente ciclo
    updateLCD();
  }
}
void runPid()
// ******* DEPENDE DE TIMER1
{
computeNow = true;
}

// ******** depende de timer 3
void RunMotor()
{
//******** es la unica funcion que funciona directamente con interrupcion
Timer1.stop();
      if (StartMotor1)
        {
        stepperX.move(VelX);
        stepperX.runSpeed();
        }
      if (StartMotor2)
        {
        stepperY.move(VelY);
        stepperY.runSpeed();
        }
      if (StartMotor3)
        {
        stepperZ.move(VelZ);
        stepperZ.runSpeed();
        }

Timer1.restart();
}

void updateLCD()
{
  // se puede poner aqui, que solo sacar mensajes si hay cambios//
  /*************************************************************/
  String mensaje = "T1 " + String(int(Input)) + " T2 " + String(int(Input2)) + " T3 " + String(int(Input3)); 
  lcd.setCursor(0, 3);
  lcd.print(mensaje);
  String mensaje2 = "T1 " + String(int(Setpoint)) + " T2 " + String(int(Setpoint2)) + " T3 " + String(int(Setpoint3)); 
  lcd.setCursor(0, 2);
  lcd.print(mensaje2);
  sacarBotellas();
}

void VerificaBotones()
{
 // **********************   USAR SOLO SI SE QUIEREN BOTONES DE ENTRAD 
 /*
  // Verificar estado del botón BotonStart1
  if (digitalRead(BotonStart1) == LOW)
  {
    StartMotor1 = !StartMotor1;
    delay(20); // Agregar un pequeño retraso para evitar múltiples detecciones del botón
  }

  // Verificar estado del botón BotonStart2
  if (digitalRead(BotonStart2) == LOW)
  {
    StartMotor2 = !StartMotor2;
    delay(20); // Agregar un pequeño retraso para evitar múltiples detecciones del botón
  }
*/

// 

  if (StartMotor1)
  {
    if (digitalRead(SENSOR_PIN1) == HIGH)
    {
      botella=botella+1;
      grabarBotella();
      Serial.print("SENSOR FILAMENTO1 ACTIVADO");
      stepperX.stop();     // Detener el motor X
      StartMotor1 = false; // paramos funcion de mover motor
      Setpoint = 0;        // apagar temperatura
      digitalWrite(ENABLE_PIN_X, HIGH);
    }
  }
  // si arrancara motor = 0 comprobar si temperatura esta activada
  // y si esta cerca de la temperatura seleccionada y filamento esta
  // dentro del sensor, activar motor

    else if((Input>(Setpoint*0.98)) && (Setpoint>0) && (digitalRead(SENSOR_PIN1) == LOW))
              {
                Motor_seleccionado=1;
                fn_on();}

  if (StartMotor2)
  {
    if (digitalRead(SENSOR_PIN2) == LOW)
    {
      botella=botella+1;
      grabarBotella();
      Serial.println("SENSOR FILAMENTO2 ACTIVADO");
      stepperY.stop();     // Detener el motor Y
      StartMotor2 = false; // paramos funcion para que no entre aqui
      Setpoint2 = 0;       // paramos la temperatura
       digitalWrite(ENABLE_PIN_Y, HIGH);
    }
  }
    else if((Input2>(Setpoint2*0.98)) && (Setpoint2>0) && (digitalRead(SENSOR_PIN2) == HIGH))
              {
                Motor_seleccionado=2;
                fn_on();}

  if (StartMotor3)
    {
    if (digitalRead(SENSOR_PIN3) == HIGH)
      {
      botella=botella+1;
      grabarBotella();
      Serial.println("SENSOR FILAMENTO3 ACTIVADO");
      stepperZ.stop();     // Detener el motor X
      StartMotor3 = false; // paramos funcion para que no entre aqui
      Setpoint3 = 0;       // paramos la temperatura
       digitalWrite(ENABLE_PIN_Z, HIGH);
      }  
    }

    else if((Input3>(Setpoint3*0.98)) && (Setpoint3>0) && (digitalRead(SENSOR_PIN3) == LOW))
              {
                Motor_seleccionado=3;
                fn_on();}
}
void grabarBotella() {
 TempBotella++;
 Serial.print ("Tempbotella ");Serial.println (TempBotella);
 Serial.print ("botellas ");Serial.println (botella);
 sacarBotellas();
 if (TempBotella >=10)
 {
  TempBotella=0;
  Serial.print ("botellas ");Serial.println (botella);
  EEPROM.put(direccion, botella);
  //EEPROM.update(direccion, botella);
 }
}

long leerBotella() {
  int temporalbotella=0;
  EEPROM.get(direccion,temporalbotella);
  return temporalbotella;
}
void sacarBotellas()
{

  //Serial.print("Botellas ");
  //Serial.println(botella);
  
  String mensaje = "Botellas " + String(long(botella));
  lcd.setCursor(0, 1);
  lcd.print(mensaje); 
}