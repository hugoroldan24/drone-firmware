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
 * analog channels (joystick axes) using Timer1 Compare Match B as a trigger. It handles automatic
 * channel switching via interrupt-driven data retrieval, pushing each 8-bit ADC result directly
 * into a circular queue for transmission tasks to consume.
 *
 * Functions:
 *   - ADC_Init
 *   - Autotrigger_Init
 *   - start_ADC_conversion
 *
 * Interrupt Service Routines:
 *   - ISR(TIMER1_COMPB_vect)
 *   - ISR(ADC_vect)
 *
 ***********************************************************************************************/
 
 
#include "common.h"
#include "circular_queue.h"
#include <avr/interrupt.h>


static volatile uint8_t channel;   /* Next channel to be read */
extern CircularQueue *transmitter_cq_ptr;

/**
 * @brief  Timer1 Compare Match B interrupt service routine.
 *         No action needed; hardware automatically clears the flag to trigger ADC auto-conversion.
 */
ISR(TIMER1_COMPB_vect)
{
	/* Clearing the interrupt flag is handled automatically by hardware */
} 
  

/**
 * @brief  ADC conversion complete interrupt service routine.
 *         Reads 8-bit result from ADCH, pushes it to circular queue, advances to next channel.
 */
ISR(ADC_vect)
{
   add_element_queue_ISR((uint8_t) ADCH,transmitter_cq_ptr); /* Store ADC result in circular queue */
   channel ++;
   channel = (channel >= NUM_ELEMENTS) ? 0 : channel;        /* Wrap around to first channel */
   ADMUX = (ADMUX & ~ADMUX_MUX) | channel;                   /* Select next channel for conversion */ 
}

/**
 * @brief  Initialize ADC with AVcc reference, 8-bit mode, prescaler for 250 kHz,
 *         Timer1 COMPB auto-trigger, and initial channel selection.
 */
void ADC_Init()
{
   PRR    &= ~(1 << PRADC);                     /* Disable ADC power reduction */
   ADCSRA |=  (1 << ADEN);                      /* Enable the ADC */
   ADCSRA |=  (1 << ADIE);                      /* Enable ADC interrupt on conversion complete */ 
   ADCSRA |=  ((1 << ADPS2)   | (1 << ADPS1));  /* Prescaler: 16 MHz / 64 = 250 kHz */
   DDRC   &=  ~((1 << PIN_X1) | (1 << PIN_Y1) |
                (1 << PIN_X2) | (1 << PIN_Y2)); /* Joystick pins as inputs */
    
   ADMUX  |=  (1 << REFS0);                     /* AVcc reference */
   ADMUX  |=  (1 << ADLAR);                     /* Left-adjust for 8-bit reads from ADCH */
   ADCSRB |=  ((1 << ADTS2) | (1 << ADTS0));    /* Auto-trigger source = Timer1 Compare Match B */
   ADCSRA |=  (1 << ADATE);                     /* Enable auto-triggering */
   ADMUX = (ADMUX & ~ADMUX_MUX) | channel;	   /* Select initial channel (0) */    
}

/**
 * @brief  Initialize Timer1 in CTC mode to generate Compare Match B interrupts
 *         at a period defined by AUTO_TRIGGER_PERIOD. Compare Match B is used
 *	   because it is the only suitable configuration when working with Timer1.
 *	   (Compare Match A can only be used when working with Timer0).	  
 */
void Autotrigger_Init()
{ 
   PRR    &= ~(1 << PRTIM1);        	 /* Disable Timer1 power reduction */
   TCNT1   =  0x0000;               	 /* Reset Timer1 counter */
   TIMSK1 |=  (1 << OCIE1B);    			 /* Enable Timer1 Compare Match B interrupt */
   TCCR1B |=  (1<< WGM12) | (1<<WGM13); /* Configure Timer1 in CTC mode where TOP = ICR1 */ 	 
   ICR1    =  AUTO_TRIGGER_PERIOD;  	 /* Set Compare value for ~0.2 ms period */
   OCR1B   =  ICR1;              		 /* Match OCR1B at TOP to trigger ISR */
   TIFR1  |=  (1 << OCF1B);	   		 /* Clear any pending Compare Match B flag */		 	
}


/**
 * @brief  Start ADC sampling by enabling Timer1 prescaler.
 *         Timer1 will now periodically trigger ADC conversions via COMPB auto-trigger.
 */
void start_ADC_conversion()
{
   TCCR1B |= (1 << CS11);   /* Prescaler /8 (2 MHz), start Timer1 */
}

