#include <ESP32Servo.h>

#include <Adafruit_VL53L0X.h>
#include <Wire.h>

#include <Adafruit_NeoPixel.h>

#include <DFRobotDFPlayerMini.h>

#include "funciones_2.h"

#define velocidad_movimiento_pinza 1

TaskHandle_t loop_2;

extern HardwareSerial Serial_2a1;
extern DFRobotDFPlayerMini DFPlayer;

uint8_t dato_recibido = 1;
uint8_t posicion =1;
bool sen_cap1 = false, sen_cap2 = false;
bool detener_robot = false;

void setup() 
{
  Inicio_RA_01();
  delay(2500);
  xTaskCreatePinnedToCore(Loop_2,    //Nombre de la función
                          "loop_2", //Nombre, puede ser cualquier cosa
                          5000,     //Tamaño de la pila
                          NULL,     //Parámetro que le quiera pasar a la tarea
                          1,        //Prioridad
                          &loop_2,   //Nombre de la tarea
                          0);       //Nucleo
}

void loop() 
{ 
  dato_recibido = Lectura_Serial_2a1();
  if(dato_recibido>10 && dato_recibido <20) 
  {
    Serial.print("Reproducir_audio: ");
    Serial.println(dato_recibido-10);
    DFPlayer.play(dato_recibido-10);
  }
  //Lectura_Bateria();
  Lectura_Sensores_Distancia();
  Tacto();
  delay(5);
}

void Loop_2(void *parameter) //Se ejecuta en el núcelo 0 **********************************
{
  Iniciar_Motores();
  while(1)
  {
    if(detener_robot)
    {
      dato_recibido=2;
      detener_robot=false;
    } 
    else
    {
      if(dato_recibido <= 7) Movimiento_Patas(dato_recibido);
      else if(dato_recibido > 7 && dato_recibido <=10) Movimiento_Pinza(dato_recibido, velocidad_movimiento_pinza);
    }
    delay(5);
    yield();
  }
}