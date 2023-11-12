

// Define el rango mínimo y máximo para el selector de valor
const int VALUE_MIN = 0;
const int VALUE_MAX = 255;

// ***************** ENCODER
#define outputA 45
#define outputB 43
#define sw 47
int aState;
int aLastState;  
int incremento=0;

//****************** MOTORES
#ifndef ENABLE_PIN_X
#define ENABLE_PIN_X 38
#endif
#ifndef ENABLE_PIN_Y
#define ENABLE_PIN_Y 56
#endif
#ifndef ENABLE_PIN_Z
#define ENABLE_PIN_Z 62
#endif

void SetupDisplay();
void Menudisplay();
void motor1();
void motor2();
void motor3();
void fn_todos();
void fn_on();
void fn_off();
void fn_mas();
void fn_men();
void fn_atras();
void fn_movimiento();
void fn_temperatura();
void selectOption();
void tmp_on(int TempT);
void tmp_off();
void tmp_mas();
void tmp_menos();

int valor = 0;
int Motor_seleccionado = 0;
int selectedValue = 0;
int TempT=220;
int TempV=300;
extern bool StartMotor1;
extern bool StartMotor2;
extern bool StartMotor3;
extern float Setpoint;
extern float Setpoint2;
extern float Setpoint3;
extern int VelX;
extern int VelY;
extern int VelZ;
extern long botella;
// Definir los pines de conexión del LCD
#define LCD_I2C_ADDR 0x27
#define LCD_COLS 20
#define LCD_ROWS 4
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

// ****** PANTALLA 0 DE INICIO  *******
//  *****  SCREEN 1 *******************
LiquidLine linea1_pant0(1, 0, "Motor1");
LiquidLine linea2_pant0(1, 1, "Motor2");
LiquidLine linea3_pant0(1, 0, "Motor3");
LiquidLine linea4_pant0(1, 1, "Todos");

//******** PANTALLA 1 *****************
// ******* SCREEN 2  ******************
LiquidLine linea1_pant1(1, 0, "TEMP      ");
LiquidLine linea2_pant1(1, 1, "MOVIMIENTO");
LiquidLine linea3_pant1(1, 0, "PRESET");
LiquidLine linea4_pant1(1, 1, "SIN USAR  ");
LiquidLine linea5_pant1(1, 0, "< atras   ");

//lineas pantalla 2
LiquidLine linea1_pant2(1, 1, "AQUA DEUS 8L 6.5mm");
LiquidLine linea2_pant2(1, 0, "AQUA DEUS 6L");
LiquidLine linea3_pant2(1, 1, "LANJARON 1L 7.5mm"); 
LiquidLine linea4_pant2(1, 0, "Pant2 lin4  ");
LiquidLine linea5_pant2(1, 1, "< atras");

//lineas pantalla 3
LiquidLine linea1_pant3(1, 0, "Pant3 lin1");
LiquidLine linea2_pant3(1, 1, "Pant3 lin2");
LiquidLine linea3_pant3(1, 0, "Pant3 lin3");
LiquidLine linea4_pant3(1, 1, "Pant3 lin4");
LiquidLine linea5_pant3(1, 0, "< atras");

//lineas pantalla 4
LiquidLine linea1_pant4(1, 0, "ON ");
LiquidLine linea2_pant4(1, 1, "OFF");
LiquidLine linea3_pant4(1, 0, "V+ ");
LiquidLine linea4_pant4(1, 1, "V- ");
LiquidLine linea5_pant4(1, 0, "< atras");

//lineas pantalla 5
// ******* screen 6 *****************
LiquidLine linea1_pant5(1, 0, "ON ");
LiquidLine linea2_pant5(1, 1, "OFF");
LiquidLine linea3_pant5(1, 0, "-5");
LiquidLine linea4_pant5(1, 1, "+5");
LiquidLine linea5_pant5(1, 0, "< atras");


//declaracion de pantallas
LiquidScreen pantalla0;
LiquidScreen pantalla1;
LiquidScreen pantalla2;
LiquidScreen pantalla3;
LiquidScreen pantalla4;
LiquidScreen pantalla5;

LiquidMenu menu(lcd);


void Menudisplay()
{
  selectOption();
// *****************  LECTURA DE ENCODER ********
aState = digitalRead(outputA); 
  if (aState != aLastState)
  { 
  
    if (digitalRead(outputB) != aState) { 
      
      incremento++;
      if(incremento>2)
      {
        incremento = 0;
        menu.switch_focus(false);
      }
  
    } else 
    {
      
      incremento++;
      if(incremento>2){
        incremento = 0;
        menu.switch_focus(true);
      }
      
    }
      menu.update();
      aLastState = aState;
  }

// ****************** CONTROL POR BOTONES ***********
/*
  int subir = digitalRead(outputA);
  int bajar = digitalRead(outputB);
  // int estadoC = digitalRead(sw);
  if (subir == LOW)
  {
    menu.switch_focus(false);
    menu.update();
    delay(10);
  }

  if (bajar == LOW)
  {
    menu.switch_focus(true);
    menu.update();
    delay(10);
  }
  */
}

void fn_ir_a_pantalla1(){
 Serial.println("Pasando a pantalla eleccion");
  menu.change_screen(2);
  menu.set_focusedLine(0);
}
// ************** NO SE USA *******
void fn_ir_a_pantalla2(){
   Serial.println("Pasando a pantalla2");
  menu.change_screen(3);
   menu.set_focusedLine(0);
}
// ************** NO SE USA *******
void fn_ir_a_pantalla3(){ 
   Serial.println("Pasando a pantalla3");
  menu.change_screen(4);
   menu.set_focusedLine(0);
}

void fn_ir_a_pantalla4(){
  Serial.println("Pasando a pantalla velocidad");
  menu.change_screen(5);
   menu.set_focusedLine(0);
}

void fn_ir_a_pantalla5(){
Serial.println("Pasando a pantalla TEMPERATURA");
  menu.change_screen(6);
   menu.set_focusedLine(0);
}


void fn_atras(){
   Serial.println("Pasando a pantalla principal");
  menu.change_screen(1);
    menu.set_focusedLine(0);
}

void fn_vacio(){ 
   Serial.println("funcion vacia");
  
}


void fn_preset1()
{
// **** AQUA DEUS 8L 
// CORTE TIRA A 6MM
TempT=225;
TempV=34;
tmp_on(TempT);
}

void fn_preset2()
{
  
}

void fn_preset3()
{
  
}

// Funciones:::::
void selectOption()
{
  if (digitalRead(sw) == LOW)
  {
    menu.call_function(1);
    delay(500);
  }
}

void motor1()
{
  Serial.println("funcion motor1");
  menu.change_screen(2);
  menu.set_focusedLine(0);
  Motor_seleccionado = 1;
}

void motor2()
{
  Serial.println("funcion motor2");
  menu.change_screen(2);
  menu.set_focusedLine(0);
  Motor_seleccionado = 2 ;
}

void motor3()
{
  Serial.println("funcion motor3");
  menu.change_screen(2);
  menu.set_focusedLine(0);
  Motor_seleccionado = 3;
}

void fn_todos()
{
  Serial.println("funcion motor todos");
  menu.change_screen(2);
  menu.set_focusedLine(0);
  Motor_seleccionado = 0;
}

//****** funcion para activar motores  *******
void fn_on()
{
  Serial.println("funcion fn_on");
  switch (Motor_seleccionado)
  {
  case 1:
    StartMotor1 = true; // iniciamos funcion de mover motor
    digitalWrite(ENABLE_PIN_X, LOW);
    ;
    break;
  case 2:
    StartMotor2 = true; // iniciamos funcion de mover motor
    digitalWrite(ENABLE_PIN_Y, LOW);
    break;
  case 3:
    StartMotor3 = true; // iniciamosfuncion de mover motor
    digitalWrite(ENABLE_PIN_Z,LOW);
    break;

  case 0:
    StartMotor1 = true; // iniciamos funcion de mover motor
    digitalWrite(ENABLE_PIN_X, LOW);

    StartMotor2 = true; // iniciamos funcion de mover motor
    digitalWrite(ENABLE_PIN_Y, LOW);

    StartMotor3 = true; // iniciamos funcion de mover motor
    digitalWrite(ENABLE_PIN_Z, LOW);

    break;
  }
}

//****************** funcion para parar motores
void fn_off()
{
  Serial.println("funcion fn_off");
  switch (Motor_seleccionado)
  {
  case 1:
    // stepperX.stop();     // Detener el motor X
    StartMotor1 = false; // paramos funcion de mover motor
    Setpoint = 0;        // apagar temperatura
    digitalWrite(ENABLE_PIN_X, HIGH);
    ;
    break;
  case 2:
    // stepperY.stop();     // Detener el motor X
    StartMotor2 = false; // paramos funcion de mover motor
    Setpoint2 = 0;       // apagar temperatura
    digitalWrite(ENABLE_PIN_Y, HIGH);

    break;
  case 3:
    // stepperZ.stop();     // Detener el motor X
    StartMotor3 = false; // paramos funcion de mover motor
    Setpoint3 = 0;       // apagar temperatura
    digitalWrite(ENABLE_PIN_Z, HIGH);
    break;

  case 0:
    // stepperX.stop();     // Detener el motor X
    StartMotor1 = false; // paramos funcion de mover motor
    Setpoint = 0;        // apagar temperatura
    digitalWrite(ENABLE_PIN_X, HIGH);
    ;

    // stepperY.stop();     // Detener el motor X
    StartMotor2 = false; // paramos funcion de mover motor
    Setpoint2 = 0;       // apagar temperatura
    digitalWrite(ENABLE_PIN_Y, HIGH);

    // stepperZ.stop();     // Detener el motor X
    StartMotor3 = false; // paramos funcion de mover motor
    Setpoint3 = 0;       // apagar temperatura
    digitalWrite(ENABLE_PIN_Z, HIGH);

    break;
  }
}
void fn_mas()
{
  Serial.println("funcion fn_mas");
  switch (Motor_seleccionado)
  {
  case 1:
    VelX = VelX + 20;
    Serial.print("VELX ;");Serial.println (VelX);
    break;
  case 2:
    VelY = VelY + 20; 
    Serial.print("VELY ;");Serial.println (VelY);
    break;
  case 3:
    VelZ = VelZ + 20; 
    Serial.print("VELZ ;");Serial.println (VelZ);
    break;

  case 0:
    VelX = VelX + 20;
    VelY = VelY + 20;
    VelZ = VelZ + 20;
    Serial.print("VELX ;");Serial.println (VelX);
    Serial.print("VELY ;");Serial.println (VelY);
    Serial.print("VELZ ;");Serial.println (VelZ);
    break;
  }
}

void fn_menos()
{
  Serial.println("funcion fn_menos");
  switch (Motor_seleccionado)
  {
  case 1:
    VelX = VelX - 20;
    Serial.print("VELX ;");Serial.println (VelX);
    break;
  case 2:
    VelY = VelY - 20;
    Serial.print("VELY ;");Serial.println (VelY);
    break;
  case 3:
    VelZ = VelZ - 20;
    Serial.print("VELZ ;");Serial.println (VelZ);
    break;

  case 0:
    VelX = VelX - 20;
    VelY = VelY - 20;
    VelZ = VelZ - 20;
    Serial.print("VELX ;");Serial.println (VelX);
    Serial.print("VELY ;");Serial.println (VelY);
    Serial.print("VELX ;");Serial.println (VelZ);
    break;
  }
}

void fn_movimiento()
{
  Serial.println("funcion movimiento");
  menu.change_screen(3); // cambiar movimientos
  menu.set_focusedLine(0);
  menu.update();
}
void fn_temperatura()
{
  Serial.println("funcion temperatura");
  menu.change_screen(4); // cambiar menu temperatura
  menu.set_focusedLine(0);
  menu.update();
}
void tmp_on(int TempT)
{
  Serial.println("funcion tmp_on");
  switch (Motor_seleccionado)
  {
  case 1:
    Setpoint = 220; // seleccion temp
    break;
  case 2:
    Setpoint2 = 220; // seleccion temp
    break;
  case 3:
    Setpoint3 = 220; // seleccion temp
    break;

  case 0:
    Setpoint = 220; // seleccion temp
    Setpoint2 = 220;
    Setpoint3 = 220;
    break;
  }
}
void tmp_off()
{
  Serial.println("funcion tmp_off");
  switch (Motor_seleccionado)
  {
  case 1:
    Setpoint = 0; // apagar temperatura
    break;
  case 2:
    Setpoint2 = 0; // apagar temperatura
    break;
  case 3:
    Setpoint3 = 0; // apagar temperatura
    break;

  case 0:
    Setpoint = 0; // apagar temperatura
    Setpoint2 = 0;
    Setpoint3 = 0;
    break;
  }
}
void tmp_mas()
{
  Serial.println("funcion tmp_mas");
  switch (Motor_seleccionado)
  {
  case 1:
    Setpoint = Setpoint + 5;
    break;
  case 2:
    Setpoint2 = Setpoint2 + 5;
    break;
  case 3:
    Setpoint3 = Setpoint3 + 5; 
    break;

  case 0:
    Setpoint = Setpoint + 5; 
    Setpoint2 = Setpoint2 + 5;
    Setpoint3 = Setpoint3 + 5;
    break;
  }
}
void tmp_menos()
{
  Serial.println("funcion tmp_menos");
  switch (Motor_seleccionado)
  {
  case 1:
    Setpoint = Setpoint - 10; // apagar temperatura
    break;
  case 2:
    Setpoint2 = Setpoint2 - 10; // apagar temperatura
    break;
  case 3:
    Setpoint3 = Setpoint3 - 10; // apagar temperatura
    break;

  case 0:
    Setpoint = Setpoint - 10;
    Setpoint2 = Setpoint2 - 10;
    Setpoint3 = Setpoint3 - 10;
    break;
  }
}