
#include "mbed.h"
#include <cstdio>
#include <cstring>
#include <cstdint>
#include "SimpleSpirit1.h"


//************************************************************

typedef struct //// Definición de estructura RADIO_DATA
{
  uint8_t          size;   
  uint8_t          srcAddr;    
  uint8_t          destAddr;                                                
  uint8_t          data[128];      
}RADIO_DATA;


//******************* 
//Cabecera de funciones
static void callback_func(int event);
void radioInit(uint8_t myAddr);
void radioSend(uint8_t addr, const char *data, uint8_t len);
uint8_t radioReceived(RADIO_DATA *data_radio);


