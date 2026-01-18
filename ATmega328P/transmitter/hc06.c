#include <stdint.h>
#include <stdio.h>
#include "usart.h"

static char* ascii_str[4];

void hc06_send_telemetry(uint8_t *buf)
{   
   USART_Send_String("Battery Voltage: ");
   
   sprintf(*ascii_str,"%u",buf[0]);
   USART_Send_String((const char*)ascii_str);
   
   USART_Send_Char('.');
   
   sprintf(*ascii_str,"%u",buf[1]);
   USART_Send_String((const char*)ascii_str);
   
   USART_Send_String(" V");
   USART_Send_Char('\n');

   USART_Send_String("Height: ");
   
   sprintf(*ascii_str,"%u",buf[2]);
   USART_Send_String((const char*)ascii_str);
   
   USART_Send_Char('.');
   
   sprintf(*ascii_str,"%u",buf[3]);
   USART_Send_String((const char*)ascii_str);
   
   USART_Send_String(" m");
   
   USART_Send_Char('\n');
   USART_Send_Char('\n');
   USART_Send_Char('\n');
}
