/***********************************************************************************************
 * telemetry.c
 *
 * Brief Description:
 * This module implements the telemetry subsystem on the receiver side. It collects telemetry
 * bytes arriving over USART into a circular queue, uses a dedicated FC sync interrupt (INT1)
 * to signal when a complete telemetry frame is ready, and then prepares ACK payloads for the
 * nRF24L01+ so that telemetry can be sent back to the transmitter in the next ACK packet.
 * It also creates an instance for the struct CircularQueue.
 *
 * Functions:
 *   - do_telemetry
 *   - init_fc_sync_irq_pin
 *
 * Interrupt Service Routines:
 *   - ISR(USART_RX_vect)
 *   - ISR(INT1_vect)
 *
 ***********************************************************************************************/


#include "common.h"
#include "delay.h"
#include "radio_receiver.h"
#include "circular_queue.h"
#include <avr/interrupt.h>

static volatile uint8_t rx_flag;

/* Create an instance for the circular queue */
CircularQueue receiver_cq={0};
receiver_cq.size  = CIRCULAR_QUEUE_SIZE_RX;
static uint8_t queue[receiver_cq.size];
receiver_cq.queue = queue;

/**
 * @brief  USART RX complete interrupt service routine.
 *         Each received byte from the flight controller is pushed into the circular queue.
 *         The FC uses DMA to send a fixed-size telemetry frame; once the frame is complete,
 *         INT1 is used to signal the receiver (see ISR(INT1_vect)).
 */
ISR(USART_RX_vect)
{
   add_element_queue_ISR((uint8_t)UDR0,&receiver_cq);   
}


/**
 * @brief  Flight controller sync interrupt (INT1) ISR.
 *         Triggered when the FC asserts the sync line after writing a complete
 *         telemetry frame into the USART/circular queue using DMA. Sets rx_flag
 *         so the telemetry task can assemble and send an ACK payload.
 */
ISR(INT1_vect)
{
   rx_flag = 1;
}


/**
 * @brief  Telemetry handler: builds and sends ACK payload when a frame is ready.
 *         When rx_flag is set (FC has signaled a complete telemetry frame), this
 *         function reads TELEM_FRAME_SIZE bytes from the circular queue into the
 *         provided buffer and loads them into the nRF24L01+ as an ACK payload.
 *
 * @param  telem_ptr  Pointer to a buffer where the telemetry frame will be stored
 *                    and from which the ACK payload will be sent.
 */
void do_telemetry(uint8_t* telem_ptr)
{
    if(rx_flag) /* This flag will only be set when the FC sends TELEM_FRAME_SIZE bytes using DMA */
    { 
       rx_flag = 0;  /* Clear sync flag: we are about to consume the frame */
       delay_us(48); /* Small delay to ensure all needed bytes are in the circular queue */ 
       (void)read_element_queue_atomic(telem_ptr,TELEM_FRAME_SIZE,&receiver_cq); /* Fetch full telemetry frame */
       send_ACK_Payload(telem_ptr,TELEM_FRAME_SIZE);                /* Load telemetry bytes into nRF24L01+ TX FIFO as ACK payload */
    }
}


/**
 * @brief  Initialize the FC sync interrupt pin (INT1).
 *         Configures INT1 as an input, sets it to trigger on a rising edge when the
 *         flight controller asserts the sync line, clears any pending INT1 flag, and
 *         enables the INT1 interrupt.
 */
void init_fc_sync_irq_pin() 
{
   DDRD  &= ~(1 << DD_INT1);               /* INT1 as input */
   EICRA |=  (1 << ISC11) | (1 << ISC10);  /* Trigger INT1 on rising edge */ 
   EIFR  |=  (1 << INTF1);			          /* Clear any pending INT1 flag */
   EIMSK |=  (1 << INT1);  		          /* Unmask INT1 interrupt */
 }

