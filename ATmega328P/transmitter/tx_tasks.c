/***********************************************************************************************
 * tx_tasks.c
 *
 * Brief Description:
 * This module defines the periodic tasks executed by the transmitter-side bare-metal scheduler.
 * It creates an instance for the struct CircularQueue.
 * It includes:
 *   - A task that pulls joystick samples from the circular queue and transmits them using the
 *     nRF24L01+ (normally NO_ACK for loss-resilient control, periodically requesting ACK to
 *     receive telemetry back).
 *   - A task that checks whether an ACK payload (telemetry) was received, reads it from the
 *     radio RX FIFO, and forwards it to the HC-06 Bluetooth module in human-readable form.
 *
 * Functions:
 *   - send_data_task
 *   - telemetry_task
 *
 ***********************************************************************************************/


#include "common.h"
#include "radio_transmitter.h"
#include "circular_queue.h"
#include "hc06.h"


extern volatile uint8_t received_telem;
extern CircularQueue *transmitter_cq_ptr;

/**
 * @brief  Scheduler task that transmits joystick data frames over the nRF24L01+.
 *         Reads the latest 4-axis joystick frame from the circular queue and sends it.
 *         Most frames are sent with NO_ACK to reduce overhead; every SEND_ACK_PERIOD_MS
 *         the task sends a frame requesting ACK so the receiver can return telemetry as
 *         an ACK payload.
 */
void send_data_task(void)
{
   static JoystickData joystick;     /* Buffer for one complete joystick frame (4 bytes) */	           

   static uint8_t time_elapsed;      /* Counts scheduler invocations */
   uint8_t success;
   time_elapsed++;                   /* 1 tick ~= 1 ms (the task period is 1 ms) */

   /* Read one full joystick frame (NUM_ELEMENTS bytes) from the circular queue atomically. */
   success = read_element_queue_atomic(joystick.axis,NUM_ELEMENTS,transmitter_cq_ptr);
   
   if(!success)
   {
      return;    /* No complete frame available yet */
   }

   /* Periodically request ACK to receive telemetry back in the ACK payload. */
   if(time_elapsed == SEND_ACK_PERIOD_MS) /* Comprobamos si ha pasado el tiempo, si es el caso pues enviamos un paquete pidiendo ACK */
   {
      time_elapsed = 0U;
      sendPaquet(joystick,ACK);     /* Send requesting ACK (telemetry may be returned) */
   }
   else{
      sendPaquet(joystick,NO_ACK);  /* Normal frames: send without ACK */
   } 
}


/**
 * @brief  Scheduler task that processes received telemetry (ACK payload) and forwards it to Bluetooth.
 *         If the radio IRQ has indicated an RX event (received_telem set), the task reads
 *         TELEM_FRAME_SIZE bytes from the nRF24L01+ RX FIFO and prints them through the HC-06.
 *
 *         This task does not block; it only checks the flag to remain scheduler-friendly.
 */
void telemetry_task(void)
{
   static uint8_t telem_data[TELEM_FRAME_SIZE];
  
   if(received_telem)  /* Flag set by INT0 ISR when IRQ asserted */
   {
      writeRegister(W_STATUS,(1<<RX_DS));
      received_telem = 0U;
      get_Telem_Data(telem_data,TELEM_FRAME_SIZE);  /* Read telemetry payload from RX FIFO */
      hc06_send_telemetry(telem_data);              /* Send formatted telemetry via USART to HC-06 */
   }
}
