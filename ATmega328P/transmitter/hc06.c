/***********************************************************************************************
 * hc06.c
 *
 * Brief Description:
 * This module formats telemetry data received from the flight controller (voltage, height) into
 * human-readable ASCII strings and transmits them over USART to a connected HC-06 Bluetooth
 * module for wireless monitoring.
 *
 * Functions:
 *   - hc06_send_telemetry
 *
 ***********************************************************************************************/


#include <stdint.h>
#include <stdio.h>
#include "usart.h"


static char ascii_str[4];


/**
 * @brief  Format and transmit telemetry data (battery voltage, height) to HC-06 Bluetooth module.
 *         Converts 4-byte binary telemetry frame into formatted ASCII strings (e.g., "3.4 V", "12.3 m")
 *         and sends them over USART with line breaks for easy serial monitor display.
 *
 * @param  buf  Pointer to 4-byte telemetry frame: [voltage_int, voltage_dec, height_int, height_dec]
 */
void hc06_send_telemetry(uint8_t *buf)
{   
   USART_Send_String("Battery Voltage: ");
   
   sprintf(ascii_str,"%u",buf[0]);             /* Integer part */
   USART_Send_String((const char*)ascii_str);
   
   USART_Send_Char('.');
   
   sprintf(ascii_str,"%u",buf[1]);              /* Decimal part */
   USART_Send_String((const char*)ascii_str);
   
   USART_Send_String(" V");
   USART_Send_Char('\n');

   USART_Send_String("Height: ");
   
   sprintf(ascii_str,"%u",buf[2]);              /* Integer part */
   USART_Send_String((const char*)ascii_str);
   
   USART_Send_Char('.');
   
   sprintf(ascii_str,"%u",buf[3]);              /* Decimal part */
   USART_Send_String((const char*)ascii_str);
   
   USART_Send_String(" m");
   
   USART_Send_Char('\n');                       /* Line breaks for readability */
   USART_Send_Char('\n');
   USART_Send_Char('\n');
}
