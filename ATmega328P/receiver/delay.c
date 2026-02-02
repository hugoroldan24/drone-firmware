/***********************************************************************************************
 * delay.c
 *
 * Brief Description:
 * This module configures hardware Timer1 on the ATmega328P to provide a simple blocking
 * microsecond delay service using CTC mode and the Output Compare A interrupt. When the
 * configured compare value (OCR1A) is reached, an ISR stops the timer and signals that the
 * requested delay has elapsed.
 *
 * Functions:
 *   - Timer1_Init
 *   - delay_us
 *
 * Interrupt Service Routines:
 *   - ISR(TIMER1_COMPA_vect)
 *
 ***********************************************************************************************/


#include "common.h"
#include <avr/interrupt.h>

static volatile uint8_t match;


/**
 * @brief  Timer1 Compare Match A interrupt service routine.
 *         Sets the match flag, stops Timer1 by clearing the prescaler bits, and
 *         resets the counter to zero so the next delay_us call starts from 0.
 */
ISR(TIMER1_COMPA_vect)
{
   match = 1;                              /* Signal that the delay interval has elapsed */
   TCCR1B &= ~((1 << CS11) | (1 << CS10)); /* Stop Timer1 by clearing prescaler bits */
   TCNT1 = 0U;                             /* Reset counter */
}


/**
 * @brief  Initialize Timer1 in CTC mode with compare match interrupt enabled.
 *         Timer1 is left stopped; delay_us will set the compare value and start it.
 */
void Timer1_Init()
{
   PRR &= ~(1 << PRTIM1);       /* Enable Timer1 module (disable power reduction) */
   TCCR1B |= (1 << WGM12);      /* CTC mode, TOP = OCR1A */
   TCNT1 = 0U;                  /* Clear counter */
   TIMSK1 |= (1 << OCIE1A);     /* Enable Timer1 Compare Match A interrupt */
}


/**
 * @brief  Busy-wait delay using Timer1 in CTC mode.
 *         Loads OCR1A with the requested tick count, starts Timer1 with a 1/64 prescaler,
 *         and spins until the compare-match ISR sets the match flag.
 *
 * @param  ticks  Number of timer ticks to wait (at 16 MHz / 64 = 250 kHz).
 */
void delay_us(uint16_t ticks)
{
   match = 0;                           /* Clear match flag before starting new delay */
   OCR1A = ticks;                       /* Set compare value for desired delay */
   TCCR1B |= (1 << CS11) | (1 << CS10); /* Start Timer1: prescaler 64 (16 MHz / 64) */
   while(!match);                       /* Block until ISR sets match flag */
   {
      /* Busy-wait; cooperative scheduler should use this carefully */
   }
}

