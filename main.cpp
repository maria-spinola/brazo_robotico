/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

 /*
Práctica 3 - Tecnologías Inalámbricas para IoT
Autora: María Spínola Lasso
 */

#include "radio_com.h"
#include "RIOSarm.h"
#include <cstdint>
#include <cstdio>
#include <cstring>

#define WAIT_TIME 300000 //Tiempo de espera para envío de query

extern SimpleSpirit1 &myspirit; //Declaración local del Spirit
RIOSarm &rios = RIOSarm::CreateInstance(PC_6,PA_12); //Instancia del objeto Riosarm


uint8_t myAddr = 0x33; //Mi dirección
uint8_t broadAddr = 0xFF; //Dirección de broadcast
bool send_flag = false; //Flag de enviado
RADIO_DATA my_radio; //Declaración de estructura RADIO_DATA
RADIO_DATA *pmy_radio = &my_radio;
static BufferedSerial serial_port(USBTX, USBRX, 115200); //Configuración Serial UART

//Declaración de variables
static uint8_t send_buf_SAR[7] = {'S','o','y',' ','S','A','R'};
static uint8_t cmd_Q_rsp[2] ={'Q',' '};
static uint8_t cmd_T_rsp[2] ={'T',' '};
static uint8_t cmd_S_rsp[2] ={'S',' '};
static char    cmd_V_rsp[20] ={'V'};

uint8_t dd_00 = 0x00; 
uint8_t dd_01 = 0x01; 
bool detected_flag = 0;
uint8_t carAddr; 
int state = 0; // state=0 -> libre / state=1 -> ocupado

Timeout t_out;
bool timeout = false;

FileHandle *mbed::mbed_override_console(int fd) //Configuración para poder utilizar printf
{
    return &serial_port;
}

static void radio_send_flag(void){ //Función para activar flag de envío
    send_flag = true;
}

void errorCallback()
{
error("RIOS UNRESPONSIVE !!\r\n");
}

void timeout_rutine(){ //Rutina de interrupcion para liberar RIOS si inactivo
    timeout = true;
}

int main()
{
    printf( "\n*Control remoto de brazo robótico*\n");
    radioInit(myAddr);
    // Define función callback…
    rios.error_event = errorCallback;

    if ( !rios.Start() ) {  //Se inicializa RIOS 
    error("NOT DETECTED !! (==> APP STOP)\r\n"); //Error si no se incializa RIOS
    }

    //bool riosDetected = rios.IsDetected(); //Comprobación de detección de RIOS
    
    printf( "\n****SAR****\n");
    rios.ResetPos(); //Reset de la posicion de RIOS
    radioSend(broadAddr, (const char*)send_buf_SAR, 7); //Se envía query en broadcast

    while(1){
    
        uint8_t d = radioReceived(&my_radio); //Función para recibir los datos
        if(d != 0){ //Si se han recibido datos
            if(my_radio.data[0] == 'q'){ //Si se recibe q
                if(state == 0){ //Si está libre
                    cmd_Q_rsp[1] = dd_00; //Se actualiza segunda posición de respuesta a 0x00
                    wait_us(WAIT_TIME); //Tiempo de espera antes de query
                    radioSend(broadAddr, (const char*)cmd_Q_rsp, 2); //Envío de Q00
                    printf("RIOS está libre");
                }
                else{
                    cmd_Q_rsp[1] = dd_01; //Se actualiza segunda posición de repsuesta a 0x00
                    wait_us(WAIT_TIME);//Tiempo de espera antes de query
                    radioSend(broadAddr, (const char*)cmd_Q_rsp, 2); //Envío de Q 01
                    printf("RIOS está bajo el control de otro remoto");
                }
            }
            else if(my_radio.data[0] == 't'){ //Si se recibe t (tomar)
                uint8_t src_Addr = my_radio.srcAddr;   //Almaceno la dirección fuente que será la destino
                if(state == 0 && my_radio.destAddr == myAddr){ //Si liberado y se dirige a mi dirección
                    cmd_T_rsp[1] = dd_01; //Control aceptado d = 0x01
                    radioSend(src_Addr, (const char*)cmd_T_rsp, 2); //Función de enviar: T 01
                    printf("Control aceptado");
                    state = 1; //Se actualiza estado del dispositivo, que deja de estar libre
                    carAddr = src_Addr; //Se actualiza la dirección de CAR
                    t_out.attach(&timeout_rutine, 60000000us); //Rutina de interrupción por timeout
                }
                else{
                    cmd_T_rsp[1] = dd_00; //Control aceptado d = 0x00
                    radioSend(src_Addr, (const char*)cmd_T_rsp, 2); //FUnción de enviar: T 00
                    printf("Control rechazado");
                }
            }
            else if(my_radio.data[0] == 'f'){ //Si se recibe f (liberar)
                uint8_t src_Addr = my_radio.srcAddr;   //Almaceno la dirección fuente que será la destino
                if(state == 1 && src_Addr == carAddr){ //Si ocupado y se dirige a mi dirección fuente = mi CAR -> se libera
                    cmd_Q_rsp[1] = dd_00; //Se actualiza segunda posición de repsuesta a 0x00
                    wait_us(WAIT_TIME); //Tiempo de espera antes de query
                    radioSend(broadAddr, (const char*)cmd_Q_rsp, 2); //Función de enviar: Q 00
                    printf("RIOS liberado");
                    state = 0; //Se actualiza estado del dispositivo, que pasa a estar libre
                    carAddr = 0x00; //Se actualiza la dirección del CAR (libre)
                    t_out.detach();//Se desactiva rutina de interrupción por timeout
                }
                else{ //Rechazada porque no estaba cogido
                    cmd_Q_rsp[1] = dd_01; //Se actualiza segunda posición de repsuesta a 0x01
                    wait_us(WAIT_TIME); //Tiempo de espera antes de query
                    radioSend(broadAddr, (const char*)cmd_T_rsp, 2); //FUnción de enviar: T 01
                    printf("Liberación rechazada");
                }
            } 
            else if(my_radio.data[0] == 's'){ //Si se recibe 's' (status motores)
                uint8_t src_Addr = my_radio.srcAddr;   //Almaceno la dirección fuente que será la destino
                if(state == 1 && src_Addr == carAddr){ //Si ocupado y se dirige a mi dirección fuente = mi CAR 
                    if(rios.status != RIOS_ON_MOVE){ //Si RIOS quieto
                        cmd_S_rsp[1] = dd_00; //Se actualiza segunda posición de repsuesta a 0x00 -> RIOS detenido
                        radioSend(src_Addr, (const char*)cmd_S_rsp, 2); //FUnción de enviar: S 00
                        printf("Status de motores: detenidos");
                        t_out.attach(&timeout_rutine, 60000000us); //Rutina de interrupción por timeout
                    }
                    else{
                        cmd_S_rsp[1] = dd_01; //Se actualiza segunda posición de repsuesta a 0x00 -> RIOS en movimiento
                        radioSend(src_Addr, (const char*)cmd_S_rsp, 2); //Función de enviar: S 01
                        printf("Status de motores: en curso");
                        t_out.attach(&timeout_rutine, 60000000us); //Rutina de interrupción por timeout
                    }
                }
            } 
            else if(my_radio.data[0] == 'v'){ //Si se recibe 'v' (version)
                uint8_t src_Addr = my_radio.srcAddr;   //Almaceno la dirección fuente que será la destino
                if(state == 1 && src_Addr == carAddr){ //Si ocupado y se dirige a mi dirección fuente = mi CAR
                    char* version = rios.version; //Se obtiene la version
 
                    for(int i=0; i<strlen(version); i++){ //Se copia la version en la respuesta
                        cmd_V_rsp[i+1] = version[i]; 
                    }
                    //printf("version: %s",version);
                    radioSend(src_Addr, cmd_V_rsp, sizeof(cmd_V_rsp)); //Función de enviar: V version
                    printf("Versión de software enviada: %s",version);
                    t_out.attach(&timeout_rutine, 60000000us); //Rutina de interrupción por timeout
                }

            }
            else if(my_radio.data[0] == 'm'){ //Si se recibe 'm' (mover motores)
                uint8_t src_Addr = my_radio.srcAddr;   //Almaceno la dirección fuente que será la destino
                if(state == 1 && src_Addr == carAddr){ //Si ocupado y se dirige a mi dirección fuente = mi CAR
                    int id_motor = my_radio.data[1]; //Se almacena id del motor
                    rios.MoveOffsetRequest(id_motor, (int8_t) my_radio.data[2], 300); //Función para mover motor: id, offset, velocidad
                    printf("Movimiento realizado: id_motor=%i, offset =%i", id_motor, (int8_t)my_radio.data[2]);
                    t_out.attach(&timeout_rutine, 60000000us); //Rutina de interrupción por timeout
                }

            }
        }
      
        if(timeout){ //Si se cumple el timeout
            timeout = false; //Se resetea el valor
            carAddr = 0x00; //Se resetea la dirección del CAR
            state = 0; //Se pasa a estado libre
            wait_us(WAIT_TIME); //Tiempo de espera para query
            cmd_Q_rsp[1] = dd_00; //Se actualiza segunda posición de repsuesta a 0x00
            radioSend(broadAddr, (const char*)cmd_Q_rsp, 2); //Envío de query
            printf("RIOS liberado por timeout");
        }                                                     
              
    }           
}