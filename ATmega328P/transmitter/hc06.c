#include <stdint.h>
#include "usart.h"

void hc06_send_telemetry(uint8_t *buf)
{
   USART_Send_String("Battery Voltage: ");
   USART_Send(&buf[0],1);
   USART_Send_Char('.');
   USART_Send(&buf[1],1);
   USART_Send_String(" V");
   USART_Send_Char('\n');

   USART_Send_String("Height: ");
   USART_Send(&buf[2],1);
   USART_Send_Char('.');
   USART_Send(&buf[3],1);
   USART_Send_String(" m");
}
