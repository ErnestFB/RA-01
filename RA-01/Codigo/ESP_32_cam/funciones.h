#ifndef FUNCIONES_H
#define FUNCIONES_H

void Configurar_Camara();           //Configura e inicia la cámara.
void Configurar_Red_WIFI();         //Configura e inicia la red WIFI.
void mover_ojos(uint8_t modo);      //Modo 0: mueve los ojos de forma lineal, modo 1: parpadea y los ojos aparecen en la posicion directamente.
void Mostrar_Cara();                //Dependiendo de la empoción, esta función cambia el color, la imagen de la boca y en algunos casos las cejas.
void Agitar_Ojos ();                //Esta función hace que la posición de los ojos alterne entre derecha e izquierda.
void Mostrar_Bateria(uint8_t num);  //Dependiendo de el parámetro de entrada, se mostrará un icono u otro de forma intermitente 3 vecces.
void Mostrar_Carta(uint8_t num);    //Dependiendo de el dato de entrada muestra una imagen u otra del juego de encontrar la carta.
void Mostrar_PPT(uint8_t num);      //Dependiendo de el dato de entrada muestra una imagen u otra del juego de piedra pepel o tijera (PPT).
void Juego_Cartas();                //Primero genera un número aleatorio, si este es distinto al previo se muestra la imagen correspondiente. Finalmente se pone la pantalla en negro.
void Juego_PPT ();                  //Primero genera un número aleatorio, si este es distinto al previo se muestra la imagen correspondiente. Finalmente se pone la pantalla en negro.
void Accion_Aleatoria();            //Cuando es llamada esta funcion se genera un número aleatorio de 0 a 39. Dependiendo del valor de este se moveran los ojos, o se parpadeará.
void Lectura_Serial_2a1();          //Lee los datos que el microcontrolador ESP 32 d1 mini envia y toma las decisiones correspondientes llamando a otras funciones o cambiando variables globales
void Boton_Pulsado_1();       //Tumbar             
void Boton_Pulsado_2();       //Caminar hacia delante
void Boton_Pulsado_3();       //Erguido
void Boton_Pulsado_4();       //Emocion nomral
void Boton_Pulsado_5();       //Rotar antihorario
void Boton_Pulsado_6();       //Parar
void Boton_Pulsado_7();       //Rotar horario
void Boton_Pulsado_8();       //Emocion feliz
void Boton_Pulsado_9();       //Frase "Hola"
void Boton_Pulsado_10();      //Caminar hacia detrás
void Boton_Pulsado_11();      //Frase "Mola"
void Boton_Pulsado_12();      //Emocion sorprendido
void Boton_Pulsado_13();      //Frase "¿Como te llamas?"
void Boton_Pulsado_14();      //Frase "Yo me llamo RA-01"
void Boton_Pulsado_15();      //Frase "Que tal el dia"
void Boton_Pulsado_16();      //Emocion asustado
void Boton_Pulsado_17();      //Frase "¿Revancha?"
void Boton_Pulsado_18();      //Frase "Genial"
void Boton_Pulsado_19();      //Frase "Adios"
void Boton_Pulsado_20();      //Emocion triste
void Boton_Pulsado_21();      //Juego Piedra Papel o Tijera, PPT
void Boton_Pulsado_22();      //Juego Cartas
void Boton_Pulsado_23();      //Baile
void Boton_Pulsado_24();      //Emocion enfadado
void Boton_Pulsado_25();      //Pinza abierta
void Boton_Pulsado_26();      //Pinza abierta a la mitad de su recorrido
void Boton_Pulsado_27();      //Cerrar pinza y mirar abajo

#endif