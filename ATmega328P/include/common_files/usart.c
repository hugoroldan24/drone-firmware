/*
 * Wireless Dual-Servo Controller
 * (c) 2025 Hugo Roldán López
 * This file is part of the project licensed under the MIT License.
 * See the LICENSE file in the root of the repository for full license text.
 */


/******************************************************************************************************
 * usart.c
 *
 * Brief Description:
 * This module provides basic USART routines on the ATmega328P.
 * It configures the USART peripheral, handles transmit interrupts, and allows sending single
 * bytes or null-terminated strings to a host terminal or another device. Useful for printing runtime data
 * (e.g Joystick values) 
 *
 * Public Functions:
 *   - void USART_Init(unsigned int ubrrn);
 *       Configures baud rate, frame format, and enables the transmitter.
 *
 *   - void USART_Send(uint8_t info);
 *       Queues a single byte for transmission via interrupt-driven method.
 *
 *   - void USART_Send_String(const char* str);
 *       Sends a null-terminated string by calling USART_Send for each character.
 *
 * Interrupt Service Routines:
 *   - ISR(USART_UDRE_vect):
 *       Triggered when the USART Data Register is empty (ready to accept next byte).
 *       Writes the next byte into UDR0, disables the UDRE interrupt, and sets a flag to
 *       indicate readiness for more data.
 *
 *****************************************************************************************************/
 
#include "common.h"
#include <avr/interrupt.h>


volatile uint8_t data;
volatile uint8_t ready = 1;	/* At startup, USART is ready to send data */



/**
 * @brief  USART Data Register Empty interrupt service routine.
 *         Sends the next byte in 'data' by writing to UDR0, then disables the interrupt
 *         until the next call to USART_Send. Sets 'ready' flag high to allow queuing
 *         of another byte.
 */
ISR(USART_UDRE_vect)
{
   UDR0 = data;		    /* Send data via USART */
   UCSR0B &= ~(1<<UDRIE0);  /* Disable UDRE interrupt until next byte is ready */
   ready = 1;  		    /* Indicate transmitter is ready for new data */
}


/**
 * @brief  Initialize the USART peripheral with the specified baud rate.
 * @param  ubrrn  Value for the UBRR0 register to set the desired baud rate.
 *                (UBRRn = F_CPU/(16 * baud) - 1 in asynchronous normal mode)
 *
 * Configures:
 *   - Baud rate registers UBRR0H:UBRR0L
 *   - Frame format: 8 data bits, no parity, 1 stop bit (8N1)
 *   - Disables double speed (normal mode)
 *   - Enables transmitter and receiver
 */
void USART_Init(unsigned int ubrrn)
{
   PRR &= ~(1<<PRUSART0);			 	    /* Disable power reduction for USART0 */	
   /* Set Baud rate */			  
   UBRR0H = (uint8_t)(ubrrn>>8);
   UBRR0L = (uint8_t)(ubrrn);
   
   UCSR0C &= (~(1<<UMSEL01) & ~(1<<UMSEL00) & ~(1<<USBS0)); /* Asynchronous mode, 1 stop bit */
   UCSR0A &= ~(1<<U2X0); 				    /* Normal speed (disable double speed) */
   
   /* Set character size to 8 bits: UCSZ01=1, UCSZ00=1, UCSZ02=0 */
   UCSR0C |= (1<< UCSZ01) | (1<<UCSZ00);  
   UCSR0B &= ~(1<<UCSZ02);   
   
   UCSR0B |= (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);			       	    /* Enable transmitter and receiver*/ 
}


/**
 * @brief  Send a single byte via USART using interrupt-driven transmit.
 * @param  info  The byte to be sent over USART.
 *
 * Waits until the previous byte is sent (ready == 1), then loads 'data' and enables
 * the UDRE interrupt, which will fire and write the byte into UDR0.
 */
void USART_Send(uint8_t* buf, uint8_t len)
{    uint8_t i;
     for(i=0;i<len;i++){
        while(!ready);	     /* Wait until previous transmission completes */
        ready = 0;	
        data = *(&buf[i]);   
        UCSR0B |= (1<<UDRIE0);  /* Enable UDRE interrupt to send next byte */ 
     }   	
}


/**
 * @brief  Send a null-terminated string via USART.
 * @param  str  Pointer to the String to transmit.
 *
 * Iterates through each character in the string until the terminating '\0' is reached,
 * passing each character to USART_Send for interrupt-driven transmission.
 */
void USART_Send_String(const char* str)
{
    while(*str){	    /* Loop until end of string '\0' */
    	USART_Send(*str++);
	}
}


 
