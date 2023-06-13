#ifndef FUNCIONES_2_H
#define FUNCIONES_2_H

void Luces_Onda(int R, int G, int B);                                                 //Enciende y apaga los LEDs en forma de onda
void Luces_Color_Fijo (int R, int G, int B);                                          //Enciende todos los LEDs a la vez y los mantiene encendido
void Inicio_Sensores_Distancia();                                                     //Inicia los sensores de distancia.
void Sensor_Cap1_Activado();                                                          //Rutina de interrupción del sensor capacitivo frontal
void Sensor_Cap2_Activado();                                                          //Rutina de interrupción del sensor capacitivo trasero
void Inicio_RA_01();                                                                  //Iniciala cadena de LEDs, las comunicaciones Serial, los sensores de distancia, el módulo de reproducción de MP3 y las interrupciones de los sensores capacitivos.
void Iniciar_Motores();                                                               //Coloca los motores en la posición inicial, una por una, realizando movimietnos bruscos en caso de que no se hayan colocado coorectamente en un inicio.
void Mover_Patas (uint8_t n_pos_inicial, uint8_t n_pos_final, const double tiempo);   //Los datos de entrada de esta función es el número de posición inicial y final y el tiempo que transcurrirá entre los distintos movimientos. Por medio de varios bucles, realiza los movimietnso correspondietes entre las posiciones indicadas.
void Movimiento_Patas(uint8_t dato_recibido);                                         //Teniendo como parametro el dato recibido que el movimiento final deseado, esta función se encargará de que los movimientos realizados sena los correctos, realizando las transiciones correspondientes.
void Movimiento_Pinza(uint8_t dato_recibido, const double tiempo);                    //Dado el dato recibido que indica la posición final y el tiempo, esta función se encargará  de mover la pinza desde la posición actual hasta la deseada de forma suave.
uint8_t Lectura_Serial_2a1();                                                         //Lee los datos que el microcontrolador ESP 32 cam envia.
void Lectura_Sensores_Distancia();                                                    //Realiza la medida de los  sensores de distancia y cambia la emoción y cambia el esttado de movimiento cuando detecta que el robot se va a caer o que ha sido levantado.
void Lectura_Bateria();                                                               //Realiza la lectura de la tensión de la batería. Si esta es baja envia un mensaje al otro microcontrolador. Si se deteta que se  ha conectado el cargador envia otro dato.
void Funcion_Sen_1();                                                                 //Esta función es activada por la rutina de interrupción del sensor capacitivo frontal. Sirve para detectar cuando se toca el robot.
void Funcion_Sen_2();                                                                 //Esta función es activada por la rutina de interrupción del sensor capacitivo trasero. Sirve para detectar cuando se toca el robot.
void Tacto();                                                                         //Esta función detecta si el robot ha sido acariciado de deatrás a delante, de delante a detrás o si se ha mantenido la mano apoyada en sobre el robot. Dependiendo de como haya sido el contacto con el robot envia un dato u otro al microcontrolador ESP 32 cam que recibe el mensaje y reacciona al mismo modificando la expresión facila del robot y la posición de sus ojos.

#endif