
// 8 bit timer counter0
#include "common.h"

volatile uint8_t time_elapsed = 0;
static volatile uint8_t count;

// Como no tenemos un timer que nos permita contar hasta 100 ms, lo que haremos es contar hasta otra cantidad y simplemente no avisar hasta
// que no hayamos llegado a los 100 ms.

// En teoria en modo CTC al llegar a OCR0A, volverá a 0 por hardware. Ese que hacemos  TCNT0 = 0U; es simplemente por si antes
// de desactivar el timer, ha contado algun valor residual, para asegurarnos que lo dejamos en 0.

// Como queremos 100 ms, pondremos 10 ciclos de 10 ms que si tenemos resolución para ello
ISR(TIMER0_COMPA_vect)
{
   if (++count == NUM_CYCLES){
      count = 0U;
      time_elapsed = 1;
      TCCR0B &= ~((1 << CS02) | (1 << CS00)); /* Deactivate Timer*/
      TCNT0 = 0U; 
   } 
}

void Timer0_Init()
{
  PRR &= ~(1 << PRTIM0);       /* Activate Timer0*/
  TCCR0A |= (1 << WGM01);      /* Set CTC mode */
  TCNT0 = 0U;
  OCR0A = OCR0A_TICKS;
  TIMSK0 |= (1 << OCIE0A);      /* Unmask OCR0A compare match interrupt */
  count = 0U;
}

void start_Timer0()
{
   TCCR0B |= (1 << CS02) | (1 << CS00);  /* Set prescaler to 16 MHz / 1024 = 15625 Hz and start counting*/   
}

