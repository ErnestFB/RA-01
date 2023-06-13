#include <ESP32Servo.h>

#include <Adafruit_VL53L0X.h>
#include <Wire.h>

#include <Adafruit_NeoPixel.h>

#include <DFRobotDFPlayerMini.h>

#define Servo__Pin_1 19
#define Servo__Pin_2 23
#define Servo__Pin_3 5
#define Servo__Pin_4 13
#define Servo__Pin_5 2
#define Servo__Pin_6 16
#define Servo__Pin_7 17
#define Servo__Pin_8 21
#define Servo__Pin_9 22

#define XSHUT_Sen_Dist_D 27
#define XSHUT_Sen_Dist_F 25
#define XSHUT_Sen_Dist_I 32

#define I2C_SDA 4
#define I2C_SCL 15

#define pin_Sen_cap1 39
#define pin_Sen_cap2 36

#define pin_Lectura_Bat 34

#define pin_Neopixel 14

#define RX_MP3 35
#define TX_MP3 33

#define RX_2a1 26
#define TX_2a1 18

#define velocidad_movimiento_1 1
#define velocidad_movimiento_R 2

#define NUM_Motores 8

//                        Levantar    CoDF    CoIT    CoIF    CoDT    CoPriPas    B_Delante(14->18)  B_Detras(18->22)
//                          0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  
const uint8_t pos_if_r[] = { 47, 47, 47, 47, 47, 35, 22, 47, 72, 47, 22, 47, 72, 47, 22, 47, 97,147, 97, 47, 97,147, 97, 47,147,147,147, 97, 47, 22, 22, 22, 35, 47, 72, 72, 72, 60, 47, 47};
const uint8_t pos_df_r[] = {156,156,156,144,131,131,131,156,181,156,131,156,180,156,131,156,106, 56,106,156,106, 56,106,156, 56,106,156,156,156,131,144,156,156,156,180,168,156,156,156,156};
const uint8_t pos_it_r[] = {137,137,137,149,162,162,162,137,112,137,162,137,112,137,162,137, 87, 37, 87,137, 87, 37, 87,137, 37, 87,137,137,137,162,149,137,137,137,112,124,137,137,137,137};
const uint8_t pos_dt_r[] = { 32, 32, 32, 32, 32, 44, 57, 32,  7, 32, 57, 32,  7, 32, 57, 32, 82,132, 82, 32, 82,132, 82, 32,132,132,132, 82, 32, 57, 57, 57, 44, 32,  7,  7,  7, 19, 32, 32};

const uint8_t pos_if_v[] = {100,100,170,170,170,145,170,145,170,170,170,170,170,145,170,170,120,170,170,170,170,170,120,170,170,170,170,120,170,170,170,170,145,170,170,170,170,145,170,100};
const uint8_t pos_df_v[] = { 80, 10, 10, 35, 10, 10, 10, 10, 10, 35, 10, 35, 10, 10, 10, 10, 10, 10, 60, 10, 60, 10, 10, 10, 10, 60, 10, 10, 10, 10, 35, 10, 10, 10, 10, 35, 10, 10, 10, 80};
const uint8_t pos_it_v[] = { 90, 90,160,135,160,160,160,160,160,135,160,135,160,160,160,160,160,160,110,160,110,160,160,160,160,110,160,160,160,160,135,160,160,160,160,135,160,160,160, 90};
const uint8_t pos_dt_v[] = { 90, 15, 15, 15, 15, 40, 15, 40, 15, 15, 15, 15, 15, 40, 15, 15, 65, 15, 15, 15, 15, 15, 65, 15, 15, 15, 15, 65, 15, 15, 15, 15, 40, 15, 15, 15, 15, 40, 15, 90};

uint8_t pos_pinza_actual=50;
uint8_t contador=0;
uint8_t dato_recibido_previo=0;
uint8_t contador_sen_1 = 0;
uint8_t contador_sen_2 = 0;
uint8_t contador_sensor_previo=150;

extern uint8_t posicion;

uint16_t nivel_bateria_previo=1000;

bool sen_1_previo = false;
bool sen_2_previo = false;

extern bool detener_robot;
extern bool sen_cap1, sen_cap2;

Servo Servo_IF_R;  //1  Izquierda frontal rotativo
Servo Servo_DF_R;  //2  Derecha frontal rotativo
Servo Servo_IT_R;  //3  Izquierda trasero rotativo
Servo Servo_DT_R;  //4  Derecha trasero rotativo
Servo Servo_IF_V;  //5  Izquierda frontal vertical
Servo Servo_DF_V;  //6  derecha frontal vertical
Servo Servo_IT_V;  //7  Izquierda trasero vertical
Servo Servo_DT_V;  //8  Derecha trasero verticcal
Servo Servo_Pinza; //9  Pinza


Adafruit_VL53L0X Sen_Dist_T = Adafruit_VL53L0X();
Adafruit_VL53L0X Sen_Dist_D = Adafruit_VL53L0X();
Adafruit_VL53L0X Sen_Dist_F = Adafruit_VL53L0X();
Adafruit_VL53L0X Sen_Dist_I = Adafruit_VL53L0X();


Adafruit_NeoPixel LED_Chain = Adafruit_NeoPixel (16, //Number of leds
                                                 pin_Neopixel,  //Pin 
                                                 NEO_GRB + NEO_KHZ800);

HardwareSerial SerialDFPlayer(1);
DFRobotDFPlayerMini DFPlayer;

HardwareSerial Serial_2a1(2);


void Luces_Onda (int R, int G, int B) //Enciende y apaga los LEDs en forma de onda.
{
  LED_Chain.setBrightness(20);
  for (int i=0; i<8; i++)
  {
    LED_Chain.setPixelColor(i,R,G,B); // Nº pixel, R,G,B 
    LED_Chain.setPixelColor(15-i,R,G,B);
    LED_Chain.show();
    delay(100);
  }
  for (int i=0; i<8; i++)
  {
    LED_Chain.setPixelColor(i,0,0,0); // Nº pixel, R,G,B
    LED_Chain.setPixelColor(15-i,0,0,0); 
    LED_Chain.show();
    delay(100);
  }
  LED_Chain.clear();
}

void Luces_Color_Fijo (int R, int G, int B) //Enciende todos los LEDs a la vez y los mantiene encendido.
{
  LED_Chain.setBrightness(20);
  for (int i=0; i<8; i++)
  {
    LED_Chain.setPixelColor(i,R,G,B); // Nº pixel, R,G,B 
    LED_Chain.setPixelColor(15-i,R,G,B);
    LED_Chain.show();
  }
  LED_Chain.show();
}

void Inicio_Sensores_Distancia()  //Inicia los sensores de distancia.
{
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("3");
  pinMode(XSHUT_Sen_Dist_D, OUTPUT);
  pinMode(XSHUT_Sen_Dist_F, OUTPUT);
  pinMode(XSHUT_Sen_Dist_I, OUTPUT);
  digitalWrite(XSHUT_Sen_Dist_D,LOW);
  digitalWrite(XSHUT_Sen_Dist_F,LOW);
  digitalWrite(XSHUT_Sen_Dist_I,LOW); delay(20);
  Serial.println("4");
  delay(50);
  if (!Sen_Dist_T.begin(0x21)) 
  {
    Serial.println("Fallo al iniciar Sen_Dist_T");
    Luces_Color_Fijo(125, 0, 0);
    while(1);
  }
  Serial.println("5");
  digitalWrite(XSHUT_Sen_Dist_D,HIGH);  delay(20);
  if (!Sen_Dist_D.begin(0x22)) 
  {
    Serial.println("Fallo al iniciar Sen_Dist_D");
    Luces_Color_Fijo(125, 0, 0);
    while(1);
  }
  digitalWrite(XSHUT_Sen_Dist_F,HIGH);  delay(20);
  if (!Sen_Dist_F.begin(0x23)) 
  {
    Serial.println("Fallo al iniciar Sen_Dist_F");
    Luces_Color_Fijo(125, 0, 0);
    while(1);
  }
  digitalWrite(XSHUT_Sen_Dist_I,HIGH);  delay(20);
  if (!Sen_Dist_I.begin(0x24)) 
  {
    Serial.println("Fallo al iniciar Sen_Dist_I");
    Luces_Color_Fijo(125, 0, 0);
    while(1);
  }
  delay(10);
  Sen_Dist_T.startRangeContinuous();
  delay(10);
  Sen_Dist_D.startRangeContinuous();
  delay(10);
  Sen_Dist_F.startRangeContinuous();
  delay(10);
  Sen_Dist_I.startRangeContinuous();
  delay(10);
  Sen_Dist_T.setMeasurementTimingBudgetMicroSeconds(25000);
  delay(10);
  Sen_Dist_D.setMeasurementTimingBudgetMicroSeconds(25000);
  delay(10);
  Sen_Dist_F.setMeasurementTimingBudgetMicroSeconds(25000);
  delay(10);
  Sen_Dist_I.setMeasurementTimingBudgetMicroSeconds(25000);
  delay(50);
}

void Sensor_Cap1_Activado() //Rutina de interrupción del sensor capacitivo frontal.
{
  sen_cap1=true;
}

void Sensor_Cap2_Activado() //Rutina de interrupción del sensor capacitivo trasero.
{
  sen_cap2=true;
}

void Inicio_RA_01()     //Iniciala cadena de LEDs, las comunicaciones Serial, los sensores de distancia, el módulo de reproducción de MP3 y las interrupciones de los sensores capacitivos.
{
  LED_Chain.begin();
  LED_Chain.setBrightness(20);
  LED_Chain.setPixelColor(0,50,50,50);  
  LED_Chain.show();
  Serial.begin(115200);
  Serial.println("i_0");
  Serial.println("i_Serial");
  LED_Chain.setPixelColor(1,50,50,50); 
  LED_Chain.show();
  SerialDFPlayer.begin(9600, SERIAL_8N1, RX_MP3, TX_MP3);
  Serial.println("i_SerialDFPlayer");
  LED_Chain.setPixelColor(2,50,50,50); 
  LED_Chain.show();
  Serial_2a1.begin(115200, SERIAL_8N1, RX_2a1, TX_2a1);
  Serial.println("i_Serial_2a1");
  LED_Chain.setPixelColor(3,50,50,50); 
  LED_Chain.show();
  Inicio_Sensores_Distancia();
  LED_Chain.setPixelColor(4,50,50,50); 
  LED_Chain.show();
  Serial.println("i_Sensores_Distancia");
  pinMode(pin_Sen_cap1,INPUT);
  pinMode(pin_Sen_cap2,INPUT);
  attachInterrupt(digitalPinToInterrupt(pin_Sen_cap1), Sensor_Cap1_Activado, RISING);  //activamos la interrupcion
  attachInterrupt(digitalPinToInterrupt(pin_Sen_cap2), Sensor_Cap2_Activado, RISING);  //activamos la interrupcion
  LED_Chain.setPixelColor(5,50,50,50); 
  LED_Chain.show();
  Serial.println("i_Sensores_Capacitivos");
  while(!DFPlayer.begin(SerialDFPlayer))
  {
    Serial.println("Iniciando DFPlayer");
  }
  LED_Chain.setPixelColor(6,50,50,50); 
  LED_Chain.show();  
  delay(250);
  DFPlayer.volume(25);  //0-30
  delay(250);
  DFPlayer.play(1);
  LED_Chain.setPixelColor(7,50,50,50); 
  LED_Chain.show();  
  Serial.println("i_DFMP3");
  Serial.println("i_finalizado");
}

void Iniciar_Motores()    //Coloca los motores en la posición inicial, una por una, realizando movimietnos bruscos en caso de que no se hayan colocado coorectamente en un inicio.
{
  Servo_IF_R.attach(Servo__Pin_1);
  Servo_DF_R.attach(Servo__Pin_2);
  Servo_IT_R.attach(Servo__Pin_3);
  Servo_DT_R.attach(Servo__Pin_4);
  Servo_IF_V.attach(Servo__Pin_5);
  Servo_DF_V.attach(Servo__Pin_6);
  Servo_IT_V.attach(Servo__Pin_7);
  Servo_DT_V.attach(Servo__Pin_8);
  Servo_Pinza.attach(Servo__Pin_9);


  Servo_IF_R.write(pos_if_r[2]);
  LED_Chain.setPixelColor(8,0,0,100);
  LED_Chain.show();
  delay(1000);
  Servo_DF_R.write(pos_df_r[2]);
  LED_Chain.setPixelColor(9,0,0,100);
  LED_Chain.show();
  delay(1000);
  Servo_IT_R.write(pos_it_r[2]);
  LED_Chain.setPixelColor(10,0,0,100);
  LED_Chain.show();
  delay(1000);
  Servo_DT_R.write(pos_dt_r[2]);
  LED_Chain.setPixelColor(11,0,0,100);
  LED_Chain.show();
  delay(1000);
  Servo_IF_V.write(pos_if_v[2]);
  LED_Chain.setPixelColor(12,0,0,100);
  LED_Chain.show();
  delay(1000);
  Servo_DF_V.write(pos_df_v[2]);
  LED_Chain.setPixelColor(13,0,0,100);
  LED_Chain.show();
  delay(1000);
  Servo_IT_V.write(pos_it_v[2]);
  LED_Chain.setPixelColor(14,0,0,100);
  LED_Chain.show();
  delay(1000);
  Servo_DT_V.write(pos_dt_v[2]);
  LED_Chain.setPixelColor(15,0,0,100);
  LED_Chain.show();
  delay(1000);
  Servo_Pinza.write(pos_pinza_actual);
  Luces_Color_Fijo (50, 125, 50);
  delay(1000);
  Servo_Pinza.detach();
}

void Mover_Patas (uint8_t n_pos_inicial, uint8_t n_pos_final, const double tiempo)  //Los datos de entrada de esta función es el número de posición inicial y final y el tiempo que transcurrirá entre los distintos movimientos.
{                                                                                   //Por medio de varios bucles, realiza los movimietnso correspondietes entre las posiciones indicadas.
  uint8_t n_pos, e_IF_R, e_DF_R, e_IT_R, e_DT_R, e_IF_V, e_DF_V, e_IT_V, e_DT_V;
  double q0[NUM_Motores], qT[NUM_Motores], a[NUM_Motores], b[NUM_Motores], c[NUM_Motores], d[NUM_Motores], t, t0, q;

  for (n_pos = n_pos_inicial; n_pos < n_pos_final; n_pos++)
  {
    q0[0] = pos_if_r[n_pos],
    q0[1] = pos_df_r[n_pos],
    q0[2] = pos_it_r[n_pos],
    q0[3] = pos_dt_r[n_pos],
    q0[4] = pos_if_v[n_pos],
    q0[5] = pos_df_v[n_pos],
    q0[6] = pos_it_v[n_pos],
    q0[7] = pos_dt_v[n_pos];

    qT[0] = pos_if_r[n_pos+1],
    qT[1] = pos_df_r[n_pos+1],
    qT[2] = pos_it_r[n_pos+1],
    qT[3] = pos_dt_r[n_pos+1],  
    qT[4] = pos_if_v[n_pos+1],
    qT[5] = pos_df_v[n_pos+1],
    qT[6] = pos_it_v[n_pos+1],
    qT[7] = pos_dt_v[n_pos+1];

    for (int i = 0; i < NUM_Motores; i++) 
    {
      a[i] = -2 * (qT[i] - q0[i]) / pow(tiempo, 3);
      b[i] = 3 * (qT[i] - q0[i]) / pow(tiempo, 2);
      c[i] = 0;
      d[i] = q0[i];
    }
    //Serial.println("Parametros calculados");
    t0 = millis() / 1000.0;
    t = 0.0;
    while (t < tiempo) 
    {
      for (int i = 0; i < NUM_Motores; i++) 
      {
        q = map(a[i] * pow(t, 3) + b[i] * pow(t, 2) + c[i] * t + d[i], 0, 180, 600, 2400);
        switch(i)
        {
          case 0:
            Servo_IF_R.writeMicroseconds(q);
          break;
          case 1:
            Servo_DF_R.writeMicroseconds(q);
          break;
          case 2:
            Servo_IT_R.writeMicroseconds(q);
          break;
          case 3:
            Servo_DT_R.writeMicroseconds(q);
          break;
          case 4:
            Servo_IF_V.writeMicroseconds(q);
          break;
          case 5:
            Servo_DF_V.writeMicroseconds(q);
          break;
          case 6:
            Servo_IT_V.writeMicroseconds(q);
          break;
          case 7:
            Servo_DT_V.writeMicroseconds(q);
          break;
        }
      }
      t = millis() / 1000.0 - t0;
    }
  }
}

void Movimiento_Patas(uint8_t dato_recibido)  //Teniendo como parametro el dato recibido que el movimiento final deseado, esta función se encargará de que los movimientos realizados sena los correctos, realizando las transiciones correspondientes.
{
  switch(dato_recibido-1)
  {
    case 0:
      Serial.println("Estado_movimiento: Tumbado");
      switch (posicion)
      {
        case 0:
        break;

        case 1:
          Mover_Patas(38,39,velocidad_movimiento_1);
          posicion=0;
        break;

        case 2:
          Mover_Patas(29,33,velocidad_movimiento_1);
          posicion=1;
        break;

        case 3:
          Mover_Patas(34,38,velocidad_movimiento_1);
          posicion=1;
        break;

        case 4:
          Mover_Patas(17,19,velocidad_movimiento_1);
          posicion=1;
        break;
      } 

    break;
    case 1:
      Serial.println("Estado_movimiento: Posición erguida");
      switch (posicion)
      {
        case 0:
          Mover_Patas(0,2,velocidad_movimiento_1);
          posicion=1;
        break;

        case 1:
        break;

        case 2:
          Mover_Patas(29,33,velocidad_movimiento_1);
          posicion=1;
        break;

        case 3:
          Mover_Patas(34,38,velocidad_movimiento_1);
          posicion=1;
        break;

        case 4:
          Mover_Patas(24,28,velocidad_movimiento_1);
          posicion=1;
        break;
      } 
    break;

    case 2:
      Serial.println("Estado_movimiento: Caminar hacia delante");
      switch (posicion)
      {
        case 0:
          Mover_Patas(0,2,velocidad_movimiento_1);
          posicion=1;
        break;

        case 1:
          Mover_Patas(2,6,velocidad_movimiento_1);
          posicion=2;
        break;

        case 2:
          Mover_Patas(6,8,velocidad_movimiento_1);
          posicion=3;
        break;

        case 3:
          Mover_Patas(8,10,velocidad_movimiento_1);
          posicion=2;
        break;

        case 4:
          Mover_Patas(24,28,velocidad_movimiento_1);
          posicion=1;
        break;
      } 
    break;

    case 3:
      Serial.println("Estado_movimiento: Caminar hacia detrás");
      switch (posicion)
      {
        case 0:
          Mover_Patas(0,2,velocidad_movimiento_1);
          posicion=1;
        break;

        case 1:
          Mover_Patas(2,6,velocidad_movimiento_1);
          posicion=2;
        break;

        case 2:
          Mover_Patas(10,12,velocidad_movimiento_1);
          posicion=3;
        break;

        case 3:
          Mover_Patas(12,14,velocidad_movimiento_1);
          posicion=2;
        break;

        case 4:
          Mover_Patas(24,28,velocidad_movimiento_1);
          posicion=1;
        break;
      }
    break;

    case 4:
      Serial.println("Estado_movimiento: Rotar antihorario");
      switch (posicion)
      {
        case 0:
          Mover_Patas(0,2,velocidad_movimiento_1);
          posicion=1;
        break;

        case 1:
          Mover_Patas(19,21,velocidad_movimiento_R);
          posicion=4;
        break;

        case 2:
          Mover_Patas(29,33,velocidad_movimiento_1);
          posicion=1;
          
        break;

        case 3:
          Mover_Patas(34,38,velocidad_movimiento_1);
          posicion=1;          
        break;

        case 4:
          Mover_Patas(21,23,velocidad_movimiento_R);
          posicion=1;
        break;
      } 
    break;

    case 5:
      Serial.println("Estado_movimiento: Rotar horario");
      switch (posicion)
      {
        case 0:
          Mover_Patas(0,2,velocidad_movimiento_1);
          posicion=1;
        break;

        case 1:
          Mover_Patas(15,17,velocidad_movimiento_R);
          posicion=4;
        break;

        case 2:
          Mover_Patas(29,33,velocidad_movimiento_1);
          posicion=1;
        break;

        case 3:
          Mover_Patas(34,38,velocidad_movimiento_1);
          posicion=1; 
        break;

        case 4:
          Mover_Patas(17,19,velocidad_movimiento_R);
          posicion=1;
        break;
      }
    break;

    case 6:
      Serial.println("Estado_movimiento: Parado");
    break;
  }
}

void Movimiento_Pinza(uint8_t dato_recibido,  const double tiempo) //Dado el dato recibido que indica la posición final y el tiempo, esta función se encargará de mover la pinza desde la posición actual hasta la deseada de forma suave.
{
  double q0, qT, a, b, c, d, t, t0, q;
  Servo_Pinza.attach(Servo__Pin_9);
  switch(dato_recibido)
  {
    case 8:
      qT = 50;
    break;

    case 9:
      qT = 90;
    break;

    case 10:
      qT = 130;
    break;
  }
  q0=pos_pinza_actual;
  pos_pinza_actual=qT;
  a = -2 * (qT - q0) / pow(tiempo, 3);
  b = 3 * (qT - q0) / pow(tiempo, 2);
  c = 0;
  d = q0;
  //Serial.println("Parametros calculados");
  t0 = millis() / 1000.0;
  t = 0.0;
  while (t < tiempo) 
  {
    q = map(a * pow(t, 3) + b * pow(t, 2) + c * t + d, 0, 180, 600, 2400);
    Servo_Pinza.writeMicroseconds(q);
    t = millis() / 1000.0 - t0;
  }
  Servo_Pinza.detach();
}

uint8_t Lectura_Serial_2a1() //Lee los datos que el microcontrolador ESP 32 cam envia.
{
  uint8_t dato_recibido = Serial_2a1.read();
  if (dato_recibido != 255)
  {
    Serial.print("Dato recibido:  ");
    Serial.println(dato_recibido);
    if (dato_recibido > 10 && dato_recibido < 20) dato_recibido_previo = 0;
    else dato_recibido_previo = dato_recibido;
    return (uint8_t)dato_recibido;
  }
    return dato_recibido_previo;
}

void Lectura_Sensores_Distancia() //Realiza la medida de los  sensores de distancia y cambia la emoción y cambia el esttado de movimiento cuando detecta que el robot se va a caer o que ha sido levantado.
{
  uint16_t dist_D, dist_I, dist_T, dist_F;

   if (Sen_Dist_D.isRangeComplete()) 
  {
    dist_D = Sen_Dist_D.readRange();
    //Serial.print(";  Sen_Dist_D : ");
    //Serial.print(dist_D);
  }
  else
  {
    Serial.print("  Sen_Dist_D falla ");
    Luces_Color_Fijo(125, 0, 0);
  }

  if (Sen_Dist_I.isRangeComplete()) 
  {
    dist_I = Sen_Dist_I.readRange();
    //Serial.print(";  Sen_Dist_I : ");
    //Serial.print(dist_I);
  }

  else
  {
    Serial.println("  Sen_Dist_I falla ");
    Luces_Color_Fijo(125, 0, 0);
  }
  
  if (Sen_Dist_T.isRangeComplete()) 
  {
    dist_T=Sen_Dist_T.readRange();
    //Serial.print("  Sen_Dist_T : ");
    //Serial.print(dist_T);
  }
  else
  {
    Serial.print("  Sen_Dist_T falla ");
    Luces_Color_Fijo(125, 0, 0);
  }
  
  if (Sen_Dist_F.isRangeComplete()) 
  {
    dist_F=Sen_Dist_F.readRange();
    //Serial.print(";  Sen_Dist_F : ");
    //Serial.println(dist_F);
  }
  else
  {
    Serial.print("  Sen_Dist_F falla ");
    Luces_Color_Fijo(125, 0, 0);
  }
  if(dist_D > 300 || dist_I > 300 || dist_T > 300 || dist_F > 300)
  {
    detener_robot=true;
    Serial_2a1.write(7);
  }
}

void Lectura_Bateria() //Realiza la lectura de la tensión de la batería. Si esta es baja envia un mensaje al otro microcontrolador. Si se deteta que se  ha conectado el cargador envia otro dato.
{
  if (contador>100)
  {
    uint16_t nivel_bateria = analogRead(pin_Lectura_Bat);
    nivel_bateria = map(nivel_bateria, 0, 4095, 0, 6600);
    if(nivel_bateria_previo == 1000) nivel_bateria_previo = nivel_bateria;
    Serial.print("Nivel Bateria: ");
    Serial.println(nivel_bateria);
    if(nivel_bateria - nivel_bateria_previo > 70)  //Se ha conectado la baetría a cargar
    {
      Serial.println("Se ha conectado el cargador");
      Serial_2a1.write(1);
    }
    if(nivel_bateria < 3400)     //Nivel de batria bajo
    {
      Serial_2a1.write(2);
      Serial.println("Se ha detectado bateria baja");
    }
    nivel_bateria_previo = nivel_bateria;
    contador=0;
  }
  else contador++;
}

void Funcion_Sen_1() //Esta función es activada por la rutina de interrupción del sensor capacitivo frontal. Sirve para detectar cuando se toca el robot.
{
  contador_sen_1++;
  sen_1_previo=true;
  sen_2_previo=false;
  if(!digitalRead(pin_Sen_cap1)) sen_cap1=false;
}

void Funcion_Sen_2() //Esta función es activada por la rutina de interrupción del sensor capacitivo trasero. Sirve para detectar cuando se toca el robot.
{
  contador_sen_2++;
  sen_1_previo=false;
  sen_2_previo=true;
  if(!digitalRead(pin_Sen_cap2)) sen_cap2=false;
}

void Tacto()  //Esta función detecta si el robot ha sido acariciado de deatrás a delante, de delante a detrás o si se ha mantenido la mano apoyada en sobre el robot. 
{             //Dependiendo de como haya sido el contacto con el robot envia un dato u otro al microcontrolador ESP 32 cam que recibe el mensaje y reacciona al mismo modificando la expresión facila del robot y la posición de sus ojos.
  if (sen_cap1 && sen_2_previo) //Previamente se habia detectado el sensor 2 y ahora se detecta el sensor 1
  {
    Serial_2a1.write(6);
    Luces_Onda (0, 125, 0);  //Luces_Onda verde
    Serial.println("Se ha acariciado el robot de detras a delante");
  }
  if (sen_cap2 && sen_1_previo) //Previamente se habia detectado el sensor 1 y ahora se detecta el sensor 2
  {
    Serial_2a1.write(5);
    Luces_Onda (126, 126, 0); //Luces_Onda amarilla
    Serial.println("Se  ha acariciado el robot de delante a detras");
  }

  if(sen_cap1) Funcion_Sen_1();
  else contador_sen_1 = 0;
  if(sen_cap2) Funcion_Sen_2();
  else contador_sen_2 = 0;
  if(contador_sen_1 > 15)
  {
    Serial_2a1.write(3);
    Luces_Onda (126, 126, 126); //Luces_Onda blanca
    Serial.println("Se ha mantenido la mano apollada sobre la parte frontal");
    contador_sen_1 = 0;
  }
  else if(contador_sen_2 > 15)
  {
    Serial_2a1.write(4);
    Luces_Onda (126, 0, 0); //Luces_Onda roja
    Serial.println("Se ha mantenido la mano apoyada sobre la parte trasera");
    contador_sen_2 = 0;
  }
  else if (sen_1_previo || sen_2_previo)
  {
    contador_sensor_previo++;
  }
  if(contador_sensor_previo > 10) 
  {
    sen_1_previo=false;
    sen_2_previo=false;
    contador_sensor_previo=0;
  }
}