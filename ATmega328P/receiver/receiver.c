/*
 * Wireless Dual-Servo Controller
 * (c) 2025 Hugo Roldán López
 * This file is part of the project licensed under the MIT License.
 * See the LICENSE file in the root of the repository for full license text.
 */
 

/***********************************************************************************************
 * receiver.c
 *
 * Brief Description:
 * This module configures all receiver-side hardware needed to operate the nRF24L01+ joystick
 * link and telemetry return path. It enables global interrupts, initializes the SPI interface,
 * configures the FC synchronization interrupt pin (used to signal that telemetry data is ready in the circular queue), 
 * initialices the RF transceiver as a receiver and starts listening for joystick frames and ACK payload exchanges.
 *  
 * Functions:
 *   - receiver_config
 *
 ***********************************************************************************************/

 
#include "spi.h"
#include "radio_receiver.h"
#include "common.h"
#include <avr/interrupt.h>


void receiver_config()
{
  sei(); 		              /* Enable global interrupts so RF IRQ (INT0) and other ISRs can fire */
  SPI_Init();		          /* Initialize SPI interface used by the nRF24L01+ */		      
  init_fc_sync_irq_pin(); /* Configure FC->receiver sync interrupt pin for telemetry-ready signaling */
  RF_Receiver_Init();     /* Initialize and configure the nRF24L01+ in receiver (PRX) mode */
  Radio_Listen();	        /* Assert CE to start listening for incoming RF packets */
}
  				  
