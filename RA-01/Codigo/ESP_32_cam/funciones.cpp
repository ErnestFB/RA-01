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

#define CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

//colores
#define NEGRO           0x0000
#define AZUL            0x001F
#define ROJO             0xF800
#define VERDE           0x07E0

#define MORADO         0xF81F
#define AMARILLO          0xFFE0  
#define BLANCO           0xFFFF

#define incremento 1

extern HardwareSerial Serial_1a2;
extern Adafruit_SSD1351 pantalla;

// SSID y contraseña para acceder a la red WIFI creada
extern const char* ssid1;
extern const char*   password;

extern uint8_t pos_ojos_actual_x;
extern uint8_t pos_ojos_actual_y;
extern uint8_t pos_ojos_deseada_x;
extern uint8_t pos_ojos_deseada_y;

extern uint8_t emocion;
extern uint8_t emocion_previa;

uint16_t color=BLANCO;

extern bool juego_cartas; 
extern bool juego_PPT;

extern bool agitar_ojos;

bool cejas=false;
bool derecha = false;

uint8_t longitud_de_la_linea = 40;
uint8_t  contador_agitar_ojos = 0;
uint8_t num_random_previo;


void  Configurar_Camara()   //Configura e inicia la cámara.
{
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0; ////zxxxxxxxxxxxxxxxx
  config.ledc_timer = LEDC_TIMER_0;     ///xxxxxxxxxxxxxxxxxx
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  //drop down frame size for higher initial frame rate
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
  s->set_vflip(s, 0);
  s->set_hmirror(s, 1);
}
void  Configurar_Red_WIFI()   //Configura e inicia la red WIFI.
{
  WiFi.softAP(ssid1,   password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
}

void mover_ojos(uint8_t modo) //Modo 0: mueve los ojos de forma lineal, modo 1: parpadea y los ojos aparecen en la posicion directamente.
{
  switch (modo)
  {
    case 0:
      if(pos_ojos_actual_y < pos_ojos_deseada_y)    //MOVIMIENTO VERTICAL (BAJAR OJOS)
      {
        pantalla.drawFastHLine(pos_ojos_actual_x-20, pos_ojos_actual_y-20, 40, NEGRO);
        pantalla.drawFastHLine(pos_ojos_actual_x+44, pos_ojos_actual_y-20, 40, NEGRO);
        pos_ojos_actual_y = pos_ojos_actual_y+incremento; 
        if(pos_ojos_actual_y > 60) longitud_de_la_linea = 40 + 60 - pos_ojos_actual_y;
        if(pos_ojos_actual_y <= 60)
        {
          pantalla.drawFastHLine(pos_ojos_actual_x-20, pos_ojos_actual_y+19, 40, color);
          pantalla.drawFastHLine(pos_ojos_actual_x+44, pos_ojos_actual_y+19, 40, color); 
        }
      }
      if(pos_ojos_actual_y > pos_ojos_deseada_y)  //MOVIMIENTO VERTICAL (SUBIR OJOS)
      {
        if(pos_ojos_actual_y <= 60)
        {
          pantalla.drawFastHLine(pos_ojos_actual_x-20, pos_ojos_actual_y+19, 40, NEGRO);
          pantalla.drawFastHLine(pos_ojos_actual_x+44, pos_ojos_actual_y+19, 40, NEGRO);   
        } 
        pos_ojos_actual_y = pos_ojos_actual_y-incremento; 
        if(pos_ojos_actual_y >= 60) longitud_de_la_linea = 40 + 60 - pos_ojos_actual_y;
        if(pos_ojos_actual_y >= 20) 
        {
          pantalla.drawFastHLine(pos_ojos_actual_x-20, pos_ojos_actual_y-20, 40, color);
          pantalla.drawFastHLine(pos_ojos_actual_x+44, pos_ojos_actual_y-20, 40, color);
        }
        
      }

      if(pos_ojos_actual_x>pos_ojos_deseada_x)    //MOVIMIENTO HORIZONTAL (IZQUIERDA)
      {
        pantalla.drawFastVLine(pos_ojos_actual_x+19, pos_ojos_actual_y-20, longitud_de_la_linea, NEGRO);
        pantalla.drawFastVLine(pos_ojos_actual_x+83, pos_ojos_actual_y-20, longitud_de_la_linea, NEGRO);
        pos_ojos_actual_x = pos_ojos_actual_x-incremento;
        if(pos_ojos_actual_x >= 20)
        {
          pantalla.drawFastVLine(pos_ojos_actual_x-20, pos_ojos_actual_y-20, longitud_de_la_linea, color);
        }
          pantalla.drawFastVLine(pos_ojos_actual_x+44, pos_ojos_actual_y-20, longitud_de_la_linea, color);
      }

      if(pos_ojos_actual_x < pos_ojos_deseada_x)    //MOVIMIENTO HORIZONTAL (DERECHA)
      {
        pantalla.drawFastVLine(pos_ojos_actual_x-20, pos_ojos_actual_y-20, longitud_de_la_linea, NEGRO);
        pantalla.drawFastVLine(pos_ojos_actual_x+44, pos_ojos_actual_y-20, longitud_de_la_linea, NEGRO);
        pos_ojos_actual_x = pos_ojos_actual_x+incremento;
        pantalla.drawFastVLine(pos_ojos_actual_x+19, pos_ojos_actual_y-20, longitud_de_la_linea, color);
        if(pos_ojos_actual_x <= 44)
        {
          pantalla.drawFastVLine(pos_ojos_actual_x+83, pos_ojos_actual_y-20, longitud_de_la_linea, color);
        }
      }
    break;

    case 1:
      if(pos_ojos_actual_x != pos_ojos_deseada_x || pos_ojos_actual_y != pos_ojos_deseada_y)
      {
        if(pos_ojos_actual_y > 60) longitud_de_la_linea = 40 + 60 - pos_ojos_actual_y;
        {
          pantalla.fillRect(0,0,128,80,NEGRO);
          pos_ojos_actual_x = pos_ojos_deseada_x;
          pos_ojos_actual_y = pos_ojos_deseada_y;
          pantalla.fillRect(pos_ojos_actual_x, pos_ojos_actual_y, 40, longitud_de_la_linea, color);
          pantalla.fillRect(pos_ojos_actual_x+64, pos_ojos_actual_y, 40, longitud_de_la_linea, color);
        }
      }
    break;
  }
}

void Mostrar_Cara()   //Dependiendo de la empoción, esta función cambia el color, la imagen de la boca y en algunos casos las cejas.
{
  if (emocion_previa != emocion)
  {
    pantalla.fillRect(pos_ojos_actual_x-32, pos_ojos_actual_y-36, 128, 16, NEGRO);
    switch(emocion)
    {
      case 0: 
        color=BLANCO;
        pantalla.fillRect(0, 80, 128, 48, NEGRO);
        pantalla.drawBitmap(0, 82, boca_normal, 128, 48, color);
        cejas=false;
        //Serial.print("Emocion: normal");
      break;

      case 1:
        color=AMARILLO;
        pantalla.fillRect(0, 80, 128, 48, NEGRO);
        pantalla.drawBitmap(0, 82, boca_contento, 128, 48, color);
        cejas=false;
        //Serial.print("Emocion: contento");
      break;

      case 2:
        color=VERDE;
        pantalla.fillRect(0, 80, 128, 48, NEGRO);
        pantalla.drawBitmap(0, 82, boca_sorprendido, 128, 48, color);
        pantalla.drawBitmap(pos_ojos_actual_x-20, pos_ojos_actual_y-36, cejas_sorprendido, 104, 16, color);
        cejas=true;
        //Serial.print("Emocion: sorprendido");
      break;

      case 3:
        color=MORADO;
        pantalla.fillRect(0, 80, 128, 48, NEGRO);
        pantalla.drawBitmap(0, 82, boca_asustado, 128, 48, color);
        cejas=false;
        //Serial.print("Emocion: asustado");
      break;

      case 4:
        color=AZUL;
        pantalla.fillRect(0, 80, 128, 48, NEGRO);
        pantalla.drawBitmap(0, 82, boca_triste, 128, 48, color);
        cejas=false;
        //Serial.print("Emocion: triste");
      break;

      case 5:
        color=ROJO;
        pantalla.fillRect(0, 80, 128, 48, NEGRO);
        pantalla.drawBitmap(0, 82, boca_enfadado, 128, 48, color);
        pantalla.drawBitmap(pos_ojos_actual_x-20, pos_ojos_actual_y-36, cejas_enfadado, 104, 16, color);
        cejas=true;
        //Serial.print("Emocion: enfadado");
      break;
    }
    emocion_previa=emocion;
    pantalla.fillRect(pos_ojos_actual_x-20, pos_ojos_actual_y-20, 40, longitud_de_la_linea, color); 
    pantalla.fillRect(pos_ojos_actual_x+44, pos_ojos_actual_y-20, 40, longitud_de_la_linea, color);
  }
  if(cejas)
  {
    if(pos_ojos_actual_x != pos_ojos_deseada_x || pos_ojos_actual_y != pos_ojos_deseada_y) 
    {
      pantalla.fillRect(pos_ojos_actual_x-32, pos_ojos_actual_y-36, 128, 16, NEGRO);
      if(emocion==2) {pantalla.drawBitmap(pos_ojos_actual_x-20, pos_ojos_actual_y-36, cejas_sorprendido, 104, 16, color);}
      if(emocion==5) {pantalla.drawBitmap(pos_ojos_actual_x-20, pos_ojos_actual_y-36, cejas_enfadado, 104, 16, color);}
    }
  }
  mover_ojos(0);
}

void Agitar_Ojos () //Esta función hace que la posición de los ojos alterne entre derecha e izquierda.
{
  if (pos_ojos_actual_x < 10) pos_ojos_deseada_x = 60; //derecha=false;
  if( pos_ojos_actual_x>54) pos_ojos_deseada_x = 5;  //derecha=true;
  contador_agitar_ojos++;
  if(contador_agitar_ojos>200)
  {
    pos_ojos_deseada_x = 32;
    pos_ojos_deseada_y = 40;
    contador_agitar_ojos=0;
    agitar_ojos=false;
  }
}

void Mostrar_Bateria(uint8_t num) //Dependiendo de el parámetro de entrada, se mostrará un icono u otro de forma intermitente 3 vecces.
{
  switch(num)
  {
    case 0:
      pantalla.drawRGBBitmap(112,0,bateria_baja,16,32);
    break;
    case 1:
      pantalla.drawRGBBitmap(112,0,bateria_media,16,32);
    break;
    case 2:
      pantalla.drawRGBBitmap(112,0,bateria_cargada,16,32);
    break;
    case 3:
      pantalla.drawRGBBitmap(112,0,bateria_cargando,16,32);
    break;
  }
}

void Mostrar_Carta (uint8_t num)  //Dependiendo de el dato de entrada muestra una imagen u otra del juego de encontrar la carta.
{ 
  switch(num)
  {
    case 0:
       pantalla.drawRGBBitmap(16,16,cartas_elefante,96,96);
    break;
    case 1:
       pantalla.drawRGBBitmap(16,16,cartas_gato,96,96);
    break;
    case 2:
       pantalla.drawRGBBitmap(16,16,cartas_leon,96,96);
    break;
    case 3:
       pantalla.drawRGBBitmap(16,16,cartas_mono,96,96);
    break;
    case 4:
       pantalla.drawRGBBitmap(16,16,cartas_pajaro,96,96);
    break;
    case 5:
       pantalla.drawRGBBitmap(16,16,cartas_perro,96,96);
    break;
    case 6:
       pantalla.drawRGBBitmap(16,16,cartas_pez,96,96);
    break;
    case 7:
       pantalla.drawRGBBitmap(16,16,cartas_raton,96,96);
    break;
    case 8:
       pantalla.drawRGBBitmap(16,16,cartas_tigre,96,96);
    break;
    case 9:
       pantalla.drawRGBBitmap(16,16,cartas_tortuga,96,96);
    break;
  }
}

void Mostrar_PPT (uint8_t num)  //Dependiendo de el dato de entrada muestra una imagen u otra del juego de piedra pepel o tijera (PPT).
{
  switch(num)
  {
    case 0:
       pantalla.drawRGBBitmap(16,16,PPT_piedra,96,96);
    break;
    case 1:
       pantalla.drawRGBBitmap(16,16,PPT_papel,96,96);
    break;
    case 2:
       pantalla.drawRGBBitmap(16,16,PPT_tijera,96,96);
    break;
  }
}

void Juego_Cartas()   //Primero genera un número aleatorio, si este es distinto al previo se muestra la imagen correspondiente. Finalmente se pone la pantalla en negro.
{
  Serial.print("Juego cartas, carta: ");
  uint8_t num_random=esp_random() % 10;
  Serial.println(num_random);
  if(num_random!=num_random_previo)
  {
    pantalla.fillRect(0, 0, 128, 128, NEGRO);
    Mostrar_Carta (num_random);
    delay(5000);
    pantalla.fillRect(0, 0, 128, 128, NEGRO);
  }
  num_random_previo=num_random;     //Se almacena en una variable global el numero previo
  emocion_previa=99;                //Dado que la emovión y la emoción previa son distintas, una vez finalice el juego se volverá a mostrar la cara del robot
}

void Juego_PPT ()     //Primero genera un número aleatorio, si este es distinto al previo se muestra la imagen correspondiente. Finalmente se pone la pantalla en negro.
{
  Serial.print("Juego PPT: ");
  delay(1000);
  uint8_t num_random=esp_random() % 3;
  //Serial.println(num_random);
  pantalla.fillRect(0, 0, 128, 128, NEGRO);
  pantalla.drawRGBBitmap(16,16,PPT,96,96);
  delay(500);
  Serial_1a2.write(19);
  delay(2000);
  pantalla.fillRect(0, 0, 128, 128, NEGRO);
  Mostrar_PPT(num_random);
  delay(4000);
  pantalla.fillRect(0, 0, 128, 128, NEGRO);
  emocion_previa=99;                //Dado que la emovión y la emoción previa son distintas, una vez finalice el juego se volverá a mostrar la cara del robot
}

void Accion_Aleatoria()   //Cuando es llamada esta funcion se genera un número aleatorio de 0 a 39. Dependiendo del valor de este se moveran los ojos, o se parpadeará.
{
  uint8_t num_random=esp_random() %40;
   if(num_random == 0)                      //Miar abajo izquierda
  {
    pos_ojos_deseada_x=20;
    pos_ojos_deseada_y=60;
  }
  else if(num_random == 1)                     //Miar izquierda
  {
    pos_ojos_deseada_x=20;
    pos_ojos_deseada_y=40;
  }
  else if(num_random == 2)                     //Miar arriba izquierda
  {
    pos_ojos_deseada_x=20;
    pos_ojos_deseada_y=20;
  }
  else if(num_random == 3)                     //Miar abajo
  {
    pos_ojos_deseada_x=32;
    pos_ojos_deseada_y=60;
  }
  else if(num_random >=4 && num_random <= 13)  //Miar centro
  {
    pos_ojos_deseada_x=32;
    pos_ojos_deseada_y=40;
  }
  else if(num_random == 14)                     //Miar arriba
  {
    pos_ojos_deseada_x=32;
    pos_ojos_deseada_y=20;
  }
  else if(num_random == 15)                     //Miar abajo derecha
  {
    pos_ojos_deseada_x=44;
    pos_ojos_deseada_y=60;
  }
  else if(num_random == 16)                     //Miar derecha
  {
    pos_ojos_deseada_x=44;
    pos_ojos_deseada_y=40;
  }
  else if(num_random == 17)                     //Miar arriba derecha
  {
    pos_ojos_deseada_x=44;
    pos_ojos_deseada_y=20;
  }
  else if(num_random > 17)                     //Parpadear
  {
    pantalla.fillRect(pos_ojos_actual_x-20, pos_ojos_actual_y-20, 104, 40, NEGRO);
    delay(250);
    pantalla.fillRect(pos_ojos_actual_x-20, pos_ojos_actual_y-20, 40, longitud_de_la_linea, color); 
    pantalla.fillRect(pos_ojos_actual_x+44, pos_ojos_actual_y-20, 40, longitud_de_la_linea, color);
  }
}

void Lectura_Serial_2a1()   //Lee los datos que el microcontrolador ESP 32 d1 mini envia y toma las decisiones correspondientes llamando a otras funciones o cambiando variables globales
{
  uint8_t dato_recibido = Serial_1a2.read();
  if (dato_recibido != 255)
  {
    Serial.print("dato_recibido: ");
    Serial.println(dato_recibido);
    switch (dato_recibido)
    {
      case 1:   //Mostrar bateria cargando
        Serial.println("Mostrar bateria cargando");
        Mostrar_Bateria(3);
        delay(500);
        pantalla.fillRect(112, 0, 16, 32, NEGRO);
        delay(500);
        Mostrar_Bateria(3);
        delay(500);
        pantalla.fillRect(112, 0, 16, 32, NEGRO);
        delay(500);
        Mostrar_Bateria(3);
        delay(500);
        pantalla.fillRect(112, 0, 16, 32, NEGRO);
      break;
      case 2:   //Mostrar bateria baja
        Serial.println("Mostrar bateria baja");
        Mostrar_Bateria(1);
        delay(500);
        pantalla.fillRect(112, 0, 16, 32, NEGRO);
        delay(500);
        Mostrar_Bateria(1);
        delay(500);
        pantalla.fillRect(112, 0, 16, 32, NEGRO);
        delay(500);
        Mostrar_Bateria(1);
        delay(500);
        pantalla.fillRect(112, 0, 16, 32, NEGRO);
      break;
      case 3:   //Se ha mantenido la mano apollada en la cabeza, emocion = normal, ojos arriba moviendose de lado a lado
        emocion = 0;
        pos_ojos_deseada_x = 60;
        pos_ojos_deseada_y=15;
        agitar_ojos=true;
      break;
      case 4:   //Se ha mantenido la mano apollada en la parte trasera, emocion = enfadado, ojos centrados
        emocion = 5;
        pos_ojos_deseada_x=32;
        pos_ojos_deseada_y=40;
      break;
      case 5:   //Se ha acariciado de delante hacia detrás, emocion = feliz, ojos arriba moviendose de lado a lado
        emocion = 1;
        pos_ojos_deseada_x = 60;
        pos_ojos_deseada_y=15;
        agitar_ojos=true;
      break;
      case 6:   //Se ha acariciado de detrás hacia delante, emocion = sorprendido, ojos centrados
      emocion = 2;
        pos_ojos_deseada_x=32;
        pos_ojos_deseada_y=40;
      break;
      case 7:   //Se ha levantado el robot y se asusta
      emocion = 3;
        pos_ojos_deseada_x=32;
        pos_ojos_deseada_y=70;
      break;
    }
  } 
}

void Boton_Pulsado_1()  //Tumbar
{
  Serial.println("Boton_Pulsado_1");
  Serial_1a2.write(1);
}

void Boton_Pulsado_2()  //Caminar hacia delante
{
  Serial.println("Boton_Pulsado_2");
  Serial_1a2.write(3);
}

void Boton_Pulsado_3()  //Erguido
{
  Serial.println("Boton_Pulsado_3");
  Serial_1a2.write(2);
}

void Boton_Pulsado_4()  //Emocion nomral
{
  Serial.println("Boton_Pulsado_4");
  emocion=0;
}

void Boton_Pulsado_5()  //Rotar antihorario
{
  Serial.println("Boton_Pulsado_5");
  pos_ojos_deseada_x=64;
  pos_ojos_deseada_y=40;
  Serial_1a2.write(5);
}
void Boton_Pulsado_6()  //Parar
{
  Serial.println("Boton_Pulsado_6");
  pos_ojos_deseada_x=32;
  pos_ojos_deseada_y=40;
  Serial_1a2.write(7);
}
void Boton_Pulsado_7()  //Rotar horario
{
  Serial.println("Boton_Pulsado_7");
  pos_ojos_deseada_x=0;
  pos_ojos_deseada_y=40;
  Serial_1a2.write(6);
}
void Boton_Pulsado_8()  //Emocion feliz
{
  Serial.println("Boton_Pulsado_8");
  emocion=1;
}
void Boton_Pulsado_9()  //Frase "Hola"
{
  Serial.println("Boton_Pulsado_9");
  Serial_1a2.write(11);
}
void Boton_Pulsado_10() //Caminar hacia detrás
{
  Serial.println("Boton_Pulsado_10");
  pos_ojos_deseada_x=64;
  pos_ojos_deseada_y=20;
  Serial_1a2.write(4);
}
void Boton_Pulsado_11() //Frase "Mola"
{
  Serial.println("Boton_Pulsado_11");
  Serial_1a2.write(12);
}
void Boton_Pulsado_12() //Emocion sorprendido
{
  Serial.println("Boton_Pulsado_12");
  emocion=2;
}
void Boton_Pulsado_13() //Frase "¿Como te llamas?"
{
  Serial.println("Boton_Pulsado_13");
  Serial_1a2.write(13);
}
void Boton_Pulsado_14() //Frase "Yo me llamo RA-01"
{
  Serial.println("Boton_Pulsado_14");
  Serial_1a2.write(14);
}
void Boton_Pulsado_15() //Frase "Que tal el dia"
{
  Serial.println("Boton_Pulsado_15");
  Serial_1a2.write(15);
}
void Boton_Pulsado_16() //Emocion asustado
{
  Serial.println("Boton_Pulsado_16");
  emocion=3;
}
void Boton_Pulsado_17() //Frase "¿Revancha?"
{
  Serial.println("Boton_Pulsado_17");
  Serial_1a2.write(16);
}
void Boton_Pulsado_18() //Frase "Genial"
{
  Serial.println("Boton_Pulsado_18");
  Serial_1a2.write(17);
}
void Boton_Pulsado_19() //Frase "Adios"
{
  Serial.println("Boton_Pulsado_19");
  Serial_1a2.write(18);
}
void Boton_Pulsado_20() //Emocion triste
{
  Serial.println("Boton_Pulsado_20");
  emocion = 4;
}
void Boton_Pulsado_21() //Juego Piedra Papel o Tijera, PPT
{
  Serial.println("Boton_Pulsado_21");
  juego_PPT=!juego_PPT;
  if(juego_PPT) juego_cartas=false;
}
void Boton_Pulsado_22() //Juego Cartas
{
  Serial.println("Boton_Pulsado_22");
  juego_cartas=!juego_cartas;
  if(juego_cartas) juego_PPT=false;
} 
void Boton_Pulsado_23() //Baile
{
  Serial.println("Boton_Pulsado_23");
  Serial_1a2.write(20);
}
void Boton_Pulsado_24() //Emocion enfadado
{
  Serial.println("Boton_Pulsado_24");
  emocion = 5;
}
void Boton_Pulsado_25() //Pinza abierta
{
  Serial.println("Boton_Pulsado_25");
  Serial_1a2.write(8);
  pos_ojos_deseada_x=32;
  pos_ojos_deseada_y=40;
} 
void Boton_Pulsado_26() //Pinza abierta a la mitad de su recorrido
{
  Serial.println("Boton_Pulsado_26");
  pos_ojos_deseada_x=32;
  pos_ojos_deseada_y=50;
  Serial_1a2.write(9);
}
void Boton_Pulsado_27() //Cerrar pinza y mirar abajo
{
  Serial.println("Boton_Pulsado_27");
  Serial_1a2.write(10);
  pos_ojos_deseada_x=32;
  pos_ojos_deseada_y=60;
}