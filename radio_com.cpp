
#include "radio_com.h"
#include <cstdio>
//#include <cstdint>

//Declaración de constantes
#define TEST_STR_LEN (128) 

//Declaración de variables
static volatile bool rx_done_flag = false; 
static volatile bool tx_done_flag = false; 
static volatile bool send_data_flag = false;

static uint8_t read_buf[TEST_STR_LEN];
static uint8_t aux_read_buf[TEST_STR_LEN]; 
uint8_t adrr = 0x33; //Dirección base: botón no pulsado
SimpleSpirit1 &myspirit = SimpleSpirit1::CreateInstance(adrr, D11, D12, D3, D9, D10, D2); //Instancia de myspirit

static void callback_func(int event)  //Función para saber si se ha recibo/transmitido
{
  if(event == SimpleSpirit1::RX_DONE) 
  {              
    rx_done_flag = 1;
  }
  else if (event == SimpleSpirit1::TX_DONE) 
  {
    tx_done_flag = 1;
  }
}

//*************************

void radioInit(uint8_t myAddr){ //Inicialización de myspirit

   myspirit.setPacketMyAddress(myAddr); //Se modifica la dirección por si se ha pulsado el botón de usuario
   myspirit.attach_irq_callback(callback_func); //Se define la rutina de interrupciones
   myspirit.on(); //Se habilita la operación de myspirit
}

void radioSend(uint8_t addr, const char *data, uint8_t len){ //Función para enviar

  //printf("\r\n***Sending a packet***\r\n");  
  
  while(myspirit.is_receiving()); /* wait for ongoing RX ends */

  uint8_t myAddres = myspirit.getPacketMyAddress(); //se obtiene la direcció local

  uint8_t payload[len+1];
  payload[0] = myAddres; //Se pone la dirección local en la posición 0 del payload

  for(int i = 0 ; i<len ; i++){
       payload[i+1] = data[i] ;  //Se copian los datos en el payload desde la posición 1
  }
   
  myspirit.setPacketDestAddress(addr); //Se define la dirección de destino
 
  myspirit.send(payload, len+1); //Se envía el paquete
  printf("\r\n***Sent a packet***\r\n\rdata = "); //Se copian datos en la estructura y se imprimen
  for(int i=0; i<len+1; i++){
    printf("%c",payload[i+1]);
  }
  //printf("%s", payload);
  printf(", srcAddr = %x, destAddr = %x, size=%d \n\r", payload[0],addr, len);
  tx_done_flag = false;
}

uint8_t radioReceived(RADIO_DATA *data_radio){ //Función para recibir

    int ret = 0;

    if(rx_done_flag){ //Si se ha recibido
       
      for(unsigned int flush_count = 0; flush_count < TEST_STR_LEN; flush_count++) read_buf[flush_count] = 0 ;/* clear the read buffer */
      
      ret = myspirit.read(read_buf, sizeof(read_buf)); //Se almacena el número de bytes que se han copiado en el búffer
      
      if(ret == 0) //si no hay nada que leer
      {
        printf("\nNothing to read\n\r");
        return 0;
      }
      else{
          for(int i=0; i < ret; i++){ //Se copian los datos
            data_radio->data[i] = read_buf[i+1];
          }
      }
      data_radio->size = ret-1; //Se almacena tamaño en la estructura
      data_radio->srcAddr = read_buf[0]; //Se almacena dirección de origen 
      data_radio->destAddr= myspirit.getPacketReceivedDestAddress(); //Se almacena dirección de destino

      printf("\r\n\n***Received a packet***\r\n\rdata = "); //Se copian datos en la estructura y se imprimen
      /*for(int i=0; i<ret; i++){
        printf("%c",data_radio->data[i]);
      }*/
      printf("%s", data_radio->data);
   
      printf(", srcAddr = %x, destAddr = %x, size=%d \n\r", data_radio->srcAddr,data_radio->destAddr, ret); //Se imprimen resto de datos
      rx_done_flag = 0;
    }

    return ret;
}
