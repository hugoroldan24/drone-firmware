/***********************************************************************************************
 * rx_tasks.c
 *
 * Brief Description:
 * This module defines the receiver-side periodic tasks that run under the bare-metal scheduler.
 * It includes a task that waits for incoming joystick frames from the nRF24L01+ (signaled via
 * IRQ/availableData), forwards them to the flight controller over USART, and a telemetry task
 * that prepares telemetry data to be sent back as ACK payloads to the transmitter.
 *
 * Functions:
 *   - receive_data_task
 *   - telemetry_task
 *
 ************************************************************************************************/

 
#include "spi.h"
#include "radio_receiver.h"
#include "common.h"
#include "telemetry.h"
#include "usart.h"


extern volatile int8_t availableData;

/**
 * @brief  Task that waits for RF joystick data and sends it to the flight controller.
 *         Blocks until the RF IRQ sets availableData, clears the nRF24L01+ RX interrupt flag,
 *         fetches the joystick payload from the RX FIFO, and transmits the decoded axes over
 *         the USART link.
 */
void receive_data_task(void)
{
  static JoystickData joystick;       /* Local joystick data buffer (union defined in common.h) */
  static uint8_t status;

  while(!availableData);					    /* Wait until nRF24L01+ external interrupt sets availableData */ 
  availableData = 0;  				 	      /* Reset flag: RX event has been handled */
     
  get_Received_Data(&joystick); 			/* Read joystick payload from RX FIFO into local struct */
  
  status = readRegister(R_STATUS);
  writeRegister(W_STATUS,status);
  
  /* Send joystick data (all axes) to the flight controller over USART */
  USART_Send(joystick.axis, NUM_ELEMENTS);
}


/**
 * @brief  Task responsible for preparing/scheduling telemetry data.
 *         Delegates to the telemetry module, which can push data into a buffer
 *         to be used as ACK payload by the radio layer.
 *
 *         Note: telem_ptr is expected to be set/managed by the telemetry subsystem
 *         (e.g., to point to the next telemetry frame to send).
 */
void telemetry_task(void)
{
  static uint8_t *telem_ptr;
  do_telemetry(telem_ptr);
}
