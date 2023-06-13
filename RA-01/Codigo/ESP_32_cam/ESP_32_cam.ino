#include <WiFi.h>

#include <soc/rtc_cntl_reg.h>

#include <dl_lib_matrix3d.h>      //Librería para el funcionamiento de la cámara
#include <img_converters.h>   

#include <esp_http_server.h>      //Librería para la creación del portal de control

#include <Adafruit_GFX.h>         //Libreríias para la pantalla
#include <Adafruit_SSD1351.h>
#include <SPI.h>

#include "app_httpd.h"            //Librerías propias
#include "funciones.h"
#include "imagenes.h"

// Dimensiones de la pantalla
#define ANCHO_PANTALLA  128
#define ALTO_PANTALLA 128

//pines pantalla
#define MOSI_PIN 12 //DIN
#define SCLK_PIN 13 //CLK
#define CS_PIN   15 //CS
#define DC_PIN   14 //MISO

//colores
#define NEGRO           0x0000
#define AZUL            0x001F
#define ROJO            0xF800
#define VERDE           0x07E0
#define MORADO          0xF81F
#define AMARILLO        0xFFE0  
#define BLANCO          0xFFFF

#define RX_1a2 16
#define TX_1a2 2


Adafruit_SSD1351 pantalla = Adafruit_SSD1351(ANCHO_PANTALLA, ALTO_PANTALLA, CS_PIN, DC_PIN, MOSI_PIN, SCLK_PIN);  

HardwareSerial Serial_1a2(1);

TaskHandle_t  Loop_2;

// SSID y contraseña para acceder a la red WIFI creada
const char* ssid1 = "RA_01";
const char*   password = "5555444433";

uint8_t pos_ojos_actual_x=32;
uint8_t pos_ojos_actual_y=40;
uint8_t pos_ojos_deseada_x=32;
uint8_t pos_ojos_deseada_y=40;

uint8_t emocion=0;
uint8_t emocion_previa=99;

bool juego_cartas = false;
bool juego_PPT = false;
bool agitar_ojos = false;

void setup() 
{
  Serial.begin(115200);
   Configurar_Camara();
   Configurar_Red_WIFI();
   Configurar_Servidor();
  delay(50);
  xTaskCreatePinnedToCore(loop_2,     //Nombre de la función
                          "loop_2",   //Nombre, puede ser cualquier cosa
                          5000,       //Tamaño de la pila
                          NULL,       //Parámetro que le quiera pasar a la tarea
                          1,          //Prioridad
                          & Loop_2,   //Nombre de la tarea
                          0);         //Nucleo
}

void loop() 
{
  delay(1);
  yield();
}

void  loop_2(void *parameter) //Se ejecuta en el núcelo 0 
{
  pantalla.begin();
  pantalla.fillScreen(NEGRO);
  Serial_1a2.begin(115200, SERIAL_8N1, RX_1a2, TX_1a2);
  Serial.println("loop_2 setup finalizado"); 
  while(1)
  {
    Lectura_Serial_2a1();
    Mostrar_Cara();
    if(juego_cartas) Juego_Cartas();
    else if (juego_PPT) Juego_PPT ();
    else if(esp_random() % 500==0) Accion_Aleatoria();
    else if(agitar_ojos) Agitar_Ojos();
    delay(5);
    yield();
  }
}