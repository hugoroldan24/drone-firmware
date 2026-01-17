/*
 * Wireless Dual-Servo Controller
 * (c) 2025 Hugo Roldán López
 * This file is part of the project licensed under the MIT License.
 * See the LICENSE file in the root of the repository for full license text.
 */
 
 
 /***********************************************************************************************************
 * main.c
 *
 * Brief Description:
 * Entry point for the wireless dual-servo transmitter. Initializes all required peripherals
 * (SPI, ADC auto-trigger, ADC, and nRF24L01+ as transmitter) and enters a loop that waits
 * for ADC conversions to complete. Once both joystick channels are sampled, it packages the
 * data and sends it via RF.
 *
 * Workflow:
 *   1. transmitter_config() initialice ADC, ADC Auto-trigger, SPI, RF module as TX and starts the conversions
 *   2. Timer1 triggers ADC conversions; each completed conversion sets sendData flag.
 *   3. ISR(ADC_vect) stores the new ADC sample in obtainedData and records lastChannel.
 *   4. In main loop, once both channels are read (lastChannel == NUM_ELEMENTS-1), call
 *      sendPaquet() to transmit the data packet via nRF24L01+.
 *
 ************************************************************************************************************/
 
#include "const.h"
#include "transmitter.h"
#include "scheduler.h"
#include "tx_tasks.h"

/**
 * @brief  Main function for the transmitter firmware.
 *         Initializes all transmitter-side peripherals and enters an infinite loop that:
 *           - Waits for ADC conversion to complete (sendData flag).
 *           - Stores the obtained sample in the joystick.axis array at index lastChannel.
 *           - When both joystick channels are sampled (lastChannel == NUM_ELEMENTS-1),
 *             calls sendPaquet() to send the full packet via the RF module.
 * @return int  Always returns 0.
 */

int main(void)
{

  (void)scheduler_add_task(send_data_task, SEND_DATA_TASK_PERIOD_MS);
  (void)scheduler_add_task(telemetry_task, TX_TELEMETRY_TASK_PERIOD_MS);

  transmitter_config();	   	        /* Initialices all the transmitter features*/

  scheduler_init();

  while(1){		
     scheduler_dispatch();
  }  
  return 0;
}


