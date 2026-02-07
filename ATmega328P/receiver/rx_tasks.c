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

static char ascii_str[4];
/**
 * @brief  Task that waits for RF joystick data and sends it to the flight controller.
 *         Blocks until the RF IRQ sets availableData, clears the nRF24L01+ RX interrupt flag,
 *         fetches the joystick payload from the RX FIFO, and transmits the decoded axes over
 *         the USART link.
 */
void receive_data_task(void)
{
  static JoystickData joystick;       /* Local joystick data buffer (union defined in common.h) */
  while(!availableData);					    /* Wait until nRF24L01+ external interrupt sets availableData */ 
  availableData = 0;  				 	      /* Reset flag: RX event has been handled */
  writeRegister(W_STATUS,(1<<RX_DS)); /* Clear RX_DS flag on nRF24L01+ (data ready IRQ) */
     
  get_Received_Data(&joystick); 			/* Read joystick payload from RX FIFO into local struct */
     
  USART_Send_String("10 ? =  ");

  sprintf(ascii_str,"%u",joystick.axis[0]);             /* Integer part */
  USART_Send_String((const char*)ascii_str);
   
  USART_Send_Char('\n');
  USART_Send_String("20 ? =  ");

  sprintf(ascii_str,"%u",joystick.axis[1]);              /* Decimal part */
  USART_Send_String((const char*)ascii_str);
  USART_Send_Char('\n');

  USART_Send_String("30 ? =  ");

  sprintf(ascii_str,"%u",joystick.axis[2]);              /* Decimal part */
  USART_Send_String((const char*)ascii_str);
  USART_Send_Char('\n');

  USART_Send_String("40 ? =  ");

  sprintf(ascii_str,"%u",joystick.axis[3]);              /* Decimal part */
  USART_Send_String((const char*)ascii_str);
   
  USART_Send_Char('\n');                       /* Line breaks for readability */
  USART_Send_Char('\n');
  USART_Send_Char('\n');

  do_telemetry(joystick.axis);
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
