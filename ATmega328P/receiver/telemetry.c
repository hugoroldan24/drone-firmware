
#include "common.h"
#include "timer1.h"
#include "radio_receiver.h"
#include "circular_queue.h"
#include <avr/interrupt.h>

static volatile uint8_t rx_flag;

ISR(USART_RX_vect)
{
    add_element_queue_ISR((uint8_t)UDR0);   
    /* Con el tema del count podriamos tener problemas en el main, osea si recibimos otro valor por UART ANTES de que el main evalue el rx_flag*/
    /* Podria pasar que el vector telemetry se sobreescribiera, dando lugar a erroes. Pero como la tarea telemetry del FC se hace cada muchos ms,*/
    /* seguro que el main tendrá tiempo de evaluar el flag y preparar el payload del ACK antes de la siguiente interrupción */  
}

ISR(INT1_vect)
{
   rx_flag = 1;
}

void do_telemetry(uint8_t* telem_ptr)
{
    if(rx_flag) /* This flag will onyl be set when the FC sends 4 bytes with the DMA*/
    { 
        rx_flag = 0; /* Aquí hacer lo de la telemetria i lo del ACK*/
        delay_us(48); /* Small delay before reading the CQ to ensure it has 4 bytes*/ 
        (void)read_element_queue_atomic(telem_ptr,TELEM_FRAME_SIZE);
        send_ACK_Payload(telem_ptr,TELEM_FRAME_SIZE);
    }
}

