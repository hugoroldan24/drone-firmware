/*
 * Wireless Dual-Servo Controller
 * (c) 2025 Hugo Roldán López
 * This file is part of the project licensed under the MIT License.
 * See the LICENSE file in the root of the repository for full license text.
 */
 
#include "spi.h"
#include "radio_receiver.h"
#include "pwm.h"
#include "common.h"
#include <avr/interrupt.h>


/**
 * @brief  Configure all receiver-side peripherals:
 *         1. Enable global interrupts.
 *         2. Initialize SPI for nRF24L01+ communication.
 *         3. Initialize PWM output (Timer1) for two servos.
 *         4. Initialize Timer0 for PWM synchronization.
 *         5. Start PWM signals on OC1A/OC1B.
 *         6. Initialize nRF24L01+ as a receiver.
 *         7. Begin listening for incoming RF packets (pull CE high).
 */
void receiver_config(){
  sei(); 		            /*Set Global Interruptions, from now, we can accept hardware interrupts*/
  SPI_Init();		        /*Initialice the SPI feature*/
  PWM_Init();  		      /*Initialice the PWM feature*/
  PWM_Timer0_Init();  	/*Initialice the Timer0*/ 
  PWM_Start();		       /*We start generating PWM signals*/
  init_interrupt_pin();
  RF_Receiver_Init();  /*We initialice the RF module as a receiver*/
  Radio_Listen();	     /*We start listening for incoming packets*/
}
  				  
