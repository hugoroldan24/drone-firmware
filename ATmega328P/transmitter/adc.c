/*
 * Wireless Dual-Servo Controller
 * (c) 2025 Hugo Roldán López
 * This file is part of the project licensed under the MIT License.
 * See the LICENSE file in the root of the repository for full license text.
 */
 
 
/***********************************************************************************************
 * adc.c
 *
 * Brief Description:
 * This module configures and manages the ADC on the ATmega328P to continuously sample multiple
 * analog channels (e.g., joystick axes) using the Timer1 Compare Match B as a trigger. It
 * handles automatic channel switching, interrupt-driven data retrieval, and makes the latest
 * ADC result available to the main application through volatile variables.
 *
 * Functions:
 *   - void ADC_Init(void);
 *       Configures ADC registers, reference, prescaler, and auto-trigger source.
 *
 *   - void Autotrigger_Init(void);
 *       Sets up Timer1 Compare Match B to generate periodic triggers for the ADC.
 *
 *   - void start_ADC_conversion(void);
 *       Starts Timer1 to begin automatic ADC conversions at the configured rate.
 *   
 *   - uint8_t readADC();
 *       Returns the obtained converted data.
 *
 *   - int8_t  getLastChannel();
 *       Returns the last channel that was converted.
 *
 * Interrupt Service Routines:
 *   - ISR(ADC_vect):
 *       Reads the 8-bit result from the ADC, signals the main loop, and advances to the
 *       next channel.
 *
 *   - ISR(TIMER1_COMPB_vect):
 *       Placeholder ISR for Timer1 Compare Match B; clearing the interrupt flag allows the
 *       ADC auto-trigger to work.
 *
 ***********************************************************************************************/
 
 
#include "common.h"
#include "circular_queue.h"
#include <avr/interrupt.h>


static volatile uint8_t channel;   /* Next channel to be read */

/**
 * @brief  Timer1 Compare Match B interrupt service routine.
 *         This ISR clears the Timer1 Compare Match B flag to allow the ADC auto-trigger
 *         to function properly. No additional action is needed here.
 */
ISR(TIMER1_COMPB_vect)
{
	/* Clearing the interrupt flag is handled automatically by hardware */
} 
  

/**
 * @brief  ADC conversion complete interrupt service routine.
 *         Reads the 8-bit result from ADCH, signals the main loop that new data is ready,
 *         saves the channel just converted, and advances to the next channel.
 */
ISR(ADC_vect)
{
  add_element_queue_ISR((uint8_t) ADCH);
  
  if(++channel >= NUM_ELEMENTS){ 
  channel = 0;   	/* Start over after last channel */
  }
  ADMUX = (ADMUX & ~ADMUX_MUX) | channel;   /* Select the next channel for conversion */ 
}

/**
 * @brief  Initialize the ADC module with AVcc as reference, 8-bit precision, prescaler settings,
 *         and auto-trigger source.
 */
void ADC_Init()
{
    PRR &= ~(1<<PRADC);                                                             /* Disable ADC power reduction */
    ADCSRA |=(1<< ADEN);                                                            /* Enable the ADC */
    ADCSRA |= (1<<ADIE);                                                            /* Enable ADC interrupt on conversion complete */ 
    ADCSRA = ADCSRA |((1<< ADPS2)|((1<<ADPS1) &~(1<<ADPS0)));                       /* Configure ADC prescaler to obtain a 250 kHz work frequency.*/
    DDRC = DDRC & (~(1 << PIN_X1) & ~(1<< PIN_Y1) & ~(1<< PIN_X2) & ~(1 << PIN_Y2));/* Configure joysticks pins as inputs*/
    
    ADMUX = ADMUX |((1<< REFS0) &~(1<<REFS1));                                      /* Set reference to AVcc (Vcc) */
    ADMUX |= (1<< ADLAR);                                                           /* Left adjust result for 8-bit precision (ADCH) */
    ADCSRB = ADCSRB  |  ((1<<ADTS2) | (1<<ADTS0));                                  /* Auto-trigger source = Timer1 Compare Match B */
    ADCSRA |= (1<<ADATE);                                                           /* Enable auto-triggering */
    ADMUX = (ADMUX & ~ADMUX_MUX) | channel;			                    			/* Select initial channel 0 */    
    /* With auto-trigger enabled, conversions start on each Timer1 Compare Match B event */
}

/**
 * @brief  Initialize Timer1 in CTC mode to generate Compare Match B interrupts
 *         at a period defined by AUTO_TRIGGER_PERIOD. Compare Match B is used
 *	   because it is the only suitable configuration when working with Timer1.
 *	   (Compare Match A can only be used when working with Timer0).	  
 */
void Autotrigger_Init()
{ 
   PRR &= ~(1<<PRTIM1);        		           /* Disable Timer1 power reduction */
   TCNT1 = 0x0000;               		  	   /* Reset Timer1 counter */
   TIMSK1 |= (1 << OCIE1B);    				   /* Enable Timer1 Compare Match B interrupt */
   TCCR1B = TCCR1B | (1<< WGM12) | (1<<WGM13); /* Configure Timer1 in CTC mode where TOP = ICR1 */ 	 
   ICR1 = AUTO_TRIGGER_PERIOD;  			   /* Set Compare value for ~0.2 ms period */
   OCR1B = ICR1;              		  	 	   /* Match OCR1B at TOP to trigger ISR */
   TIFR1 |= (1 << OCF1B);	   				   /* Clear any pending Compare Match B flag */		 	
}

/**
 * @brief  Start Timer1 with prescaler = 8 (CPU/8 = 2 MHz) to begin auto-triggered
 *         ADC conversions. Once this is set, Timer1 will periodically generate
 *         Compare Match B interrupts.
 */
void start_ADC_conversion()
{
   TCCR1B |= (1<<CS11);   /* Set prescaler to 8 and start Timer1 */
}

