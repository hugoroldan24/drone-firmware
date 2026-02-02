/*
 * Wireless Dual-Servo Controller
 * (c) 2025 Hugo Roldán López
 * This file is part of the project licensed under the MIT License.
 * See the LICENSE file in the root of the repository for full license text.
 */


 /***********************************************************************************************
 * usart.c
 *
 * Brief Description:
 * This module provides interrupt-driven USART transmission on the ATmega328P for sending
 * joystick data (to FC), telemetry (to Bluetooth HC-06), or debug output. Uses UDRE interrupt
 * for non-blocking TX; receiver is also enabled for telemetry from FC. Supports single bytes,
 * buffers, and null-terminated strings.
 *
 * Functions:
 *   - USART_Init
 *   - USART_Send
 *   - USART_Send_Char
 *   - USART_Send_String
 *
 * Interrupt Service Routines:
 *   - ISR(USART_UDRE_vect)
 *   - ISR(USART_RX_vect) 
 *
 ***********************************************************************************************/

 
#include "common.h"
#include <avr/interrupt.h>


volatile uint8_t data;
volatile uint8_t ready = 1; /* Transmitter ready flag (1 = idle) */


/**
 * @brief  USART Data Register Empty (UDRE) interrupt service routine.
 *         Loads pending data into UDR0, disables UDRE interrupt until next byte queued,
 *         and sets ready flag.
 */
ISR(USART_UDRE_vect)
{
   UDR0 = data;		        /* Send data via USART */
   UCSR0B &= ~(1 << UDRIE0); /* Disable UDRE interrupt until next byte is ready */
   ready = 1;  		        /* Indicate transmitter is ready for new data */
}


/**
 * @brief  Initialize USART with specified baud rate (normal async mode, 8N1).
 *         Enables TX/RX + RX complete interrupt.
 *
 * @param  ubrrn  Baud rate register value: F_CPU / (16 * baud) - 1.
 */
void USART_Init(unsigned int ubrrn)
{
   PRR &= ~(1<<PRUSART0);			 	  /* Enable USART0 power */

   /* Set Baud rate */			  
   UBRR0H = (uint8_t)(ubrrn>>8);
   UBRR0L = (uint8_t)(ubrrn);
   
   /* Async normal mode, 1 stop bit, no double speed */
   UCSR0C &= ~((1 << UMSEL01) | (1 << UMSEL00) | (1 << USBS0)); 
   UCSR0A &= ~(1 << U2X0); 				  
   
  /* 8 data bits */
   UCSR0C |=  (1 << UCSZ01) | (1 << UCSZ00);  
   UCSR0B &= ~(1 << UCSZ02);   
   
   /* Enable TX, RX, and RX complete interrupt */
   UCSR0B |= (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);			       	   
}


/**
 * @brief  Blocking send of a multi-byte buffer via interrupt-driven TX.
 *         Waits for transmitter to be ready before queuing each byte.
 *
 * @param  buf  Buffer of bytes to send.
 * @param  len  Number of bytes.
 */
void USART_Send(uint8_t* buf, uint8_t len)
{    uint8_t i;
     for(i=0;i<len;i++){
        while(!ready);	       /* Wait for previous TX complete */
        ready = 0;	
        data = buf[i];         /* Queue next byte */
        UCSR0B |= (1<<UDRIE0); /* Enable UDRE to transmit */
     }   	
}


/**
 * @brief  Blocking send of a single character via interrupt-driven TX.
 *
 * @param  c  Character to send.
 */
void USART_Send_Char(char c)
{
   while(!ready);	         /* Wait FOR previous transmission to complete */
   ready = 0;	
   data = (uint8_t) c;   
   UCSR0B |= (1 << UDRIE0); /* Trigger TX */  
}

/**
 * @brief  Blocking send of a null-terminated string.
 *
 * @param  str  Pointer to string.
 */
void USART_Send_String(const char* str)
{
   while(*str)
   {	    
      USART_Send_Char(*str++);  /* Send char-by-char */
	}
}


 
