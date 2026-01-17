/*
 * Wireless Dual-Servo Controller
 * (c) 2025 Hugo Roldán López
 * This file is part of the project licensed under the MIT License.
 * See the LICENSE file in the root of the repository for full license text.
 */
 
 
#include "adc.h"
#include "spi.h"
#include "common.h"
#include "usart.h"
#include "radio_transmitter.h"
#include <avr/interrupt.h>


/**
 * @brief  Configure all transmitter-side peripherals:
 *         1. Enable global interrupts.
 *         2. Initialize SPI for nRF24L01+ communication.
 *         3. Initialize Timer1 as ADC auto-trigger source.
 *         4. Initialize ADC for joystick sampling.
 *         5. Initialize nRF24L01+ as a transmitter.
 *         6. Start continuous ADC conversions.
 */
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
