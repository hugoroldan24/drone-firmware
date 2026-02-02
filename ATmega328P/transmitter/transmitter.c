/*
 * Wireless Dual-Servo Controller
 * (c) 2025 Hugo Roldán López
 * This file is part of the project licensed under the MIT License.
 * See the LICENSE file in the root of the repository for full license text.
 */
 
 
/***********************************************************************************************
 * transmitter.c
 *
 * Brief Description:
 * This module configures all transmitter-side peripherals required to sample joystick inputs
 * and transmit them wirelessly using an nRF24L01+ in PTX mode. It enables global interrupts,
 * initializes SPI for radio communication, configures Timer1 as the ADC auto-trigger source,
 * initializes the ADC to sample multiple joystick channels into the circular queue, initializes
 * the RF transmitter, starts continuous ADC conversions, and initializes USART.
 *
 * Functions:
 *   - transmitter_config
 *
 ***********************************************************************************************/

 
#include "adc.h"
#include "spi.h"
#include "common.h"
#include "usart.h"
#include "radio_transmitter.h"
#include <avr/interrupt.h>


void transmitter_config()
{
  sei();		  
  SPI_Init();		  
  Autotrigger_Init();     
  ADC_Init(); 		  
  RF_Transmitter_Init();  	  
  start_ADC_conversion();
  USART_Init(UBRR);
}
