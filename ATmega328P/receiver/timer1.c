
#include "common.h"

static volatile uint8_t match;

ISR(TIMER1_COMPA_vect) {
   match = 1;
   TCCR1B &= ~((1 << CS11) | (1 << CS10)); /* Deactivate Timer*/
   TCNT1 = 0U;
}


void Timer1_Init()
{
  PRR &= ~(1 << PRTIM1);       /* Activate Timer1 */
  TCCR1B |= (1 << WGM12);      /* Set CTC mode */
  TCNT1 = 0U;
  TIMSK1 |= (1 << OCIE1A);      /* Unmask OCR1A compare match interrupt */
}

void delay_us(uint16_t ticks)
{
    OCR1A = ticks;
    TCCR1B |= (1 << CS11) | (1 << CS10);  /* Set prescaler to 16 MHz / 64 = 250 kHz and start counting*/ 
    while(!match);
}

