/*
 * Wireless Dual-Servo Controller
 * (c) 2025 Hugo Roldán López
 * This file is part of the project licensed under the MIT License.
 * See the LICENSE file in the root of the repository for full license text.
 */
 
 
/***********************************************************************************************************
 * pwm.c
 *
 * Brief Description:
 * This module configures and manages Timer1 (16 bits) and Timer0 (8 bits) on the ATmega328P to generate and
 * synchronize PWM signals for two servomotors. Timer1 is set up in Fast PWM mode with ICR1
 * defining the 20 ms period (50 Hz). The OCR1A and OCR1B registers control each servo's pulse
 * width. Timer0 runs in CTC mode to update OCR1A/B every 15 ms (after three 5 ms intervals),
 * ensuring servo updates occur at the correct 50 Hz rate without excessive register writes.
 * Otherwise, we would write OCR1A/B many times but only once every 20 ms, we would fetch the registers
 *
 * Public Functions:
 *   - void PWM_Init(void);
 *       Configures Timer1 for Fast PWM, sets output pins for OC1A/OC1B, and initializes
 *       pulse widths to the idle state.
 *
 *   - void PWM_Timer0_Init(void);
 *       Configures Timer0 in CTC mode with OCR0A to generate 5 ms interrupts for synchronizing
 *       servo updates.
 *
 *   - void PWM_Start(void);
 *       Starts Timer1 with a prescaler of 256 to generate the PWM waveform and starts Timer0
 *       with a prescaler of 1024 to generate synchronization interrupts.
 *
 *   - void Convert_Value_PWM(uint8_t Xaxis, uint8_t Yaxis, volatile uint16_t *converted_valueA,
 *                             volatile uint16_t *converted_valueB);
 *       Performs linear interpolation on joystick 8-bit values to compute appropriate OCR1A/B
 *       pulse widths for the servos.
 *
 * Interrupt Service Routines:
 *   - ISR(TIMER0_COMPA_vect):
 *       Triggered every 5 ms by Timer0 Compare Match A. On every third interrupt (≈15 ms),
 *       updates OCR1A and OCR1B with the latest servo positions, then resets the interrupt
 *       counter to synchronize PWM updates at 50 Hz.
 *
 ***********************************************************************************************************/
 
  
#include "common.h"
#include "pwm.h"
#include <avr/interrupt.h>

static  Servo servos;                     /* Declares the struct servos (defined in pwm.h file).*/
  
static volatile int8_t interrupt_count = 0; 

 
/**
 * @brief  Timer0 Compare Match A interrupt service routine.
 *         Increments a counter each 5 ms. On the third invocation (~15 ms), writes the latest
 *         servo pulse widths into OCR1A and OCR1B, then resets the counter to -1 so that
 *         next interrupt makes it zero, synchronizing Timer1 and Timer0.
 */
ISR(TIMER0_COMPA_vect)
{
   if(++interrupt_count == 3){	/* Every 15 ms (3 × 5 ms) */
	OCR1A = servos.sA;  	    /* Refresh OCR1A for servo A */
	OCR1B = servos.sB;	    	/* Refresh OCR1B for servo B */
	interrupt_count = -1;	    /* Reset so next interrupt sets it to 0 */
   }
}


/**
 * @brief  Initialize Timer1 for Fast PWM on OC1A and OC1B to drive two servos.
 *	   - Sets the default value the sermotors have
 *         - Activates Timer1
 *         - Configures PB1 (OC1A) and PB2 (OC1B) as outputs
 *         - Sets Fast PWM mode with ICR1 = PWM_PERIOD to yield a 20 ms period
 *         - Configures non-inverting output on OC1A/OC1B
 */
void PWM_Init()
{
  servos.sA = IDLE_STATE;   				   /* Default value when the joysticks are at rest */
  servos.sB = IDLE_STATE; 
  
  PRR &= ~(1<<PRTIM1);        			   	   /* Activate Timer1 */
  DDRB |= (1<<DD_OC1A) | (1<<DD_OC1B);     	   /* OC1A (PB1) and OC1B (PB2) as outputs */
  TCNT1 = 0x0000;               			   /* Reset Timer1 counter */
  TCCR1B |= (1 << WGM13) | (1 << WGM12); 	   /* Fast PWM mode 14: TOP = ICR1 */
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1) 	   /* Non-inverting mode on OC1A/OC1B (clear when matching upcounting and set when reaching bottom */
  | (1 << WGM11);  					   		   /* Part of Fast PWM mode */
  ICR1 = PWM_PERIOD;  	    			 	   
}


/**
 * @brief  Initialize Timer0 in CTC mode to generate synchronization interrupts every 5 ms.
 *         - Activates Timer0
 *         - Configures CTC mode with OCR0A as TOP
 *         - Enables Compare Match A interrupt
 *         - Clears any pending flag
 *         - Sets OCR0A = TIMER0_PERIOD for a 5 ms interval
 */
void PWM_Timer0_Init()
{
  PRR &= ~(1<<PRTIM0);                    /* Activate Timer0 */
  TCCR0A |=(1<<WGM01);                    /* CTC mode */
  TIMSK0 |= (1<<OCIE0A);				  /* Unmask Compare Match A interrupt */
  TIFR0 |= (1<< OCF0A);					  /* Clear any pending interrupt flag */
  TCNT0 = 0x00;						      /* Reset Timer0 counter */
  OCR0A = TIMER0_PERIOD;         		  /* Set Compare value for 5 ms */
}


/**
 * @brief  Start PWM generation and synchronization timers:
 *         - Timer1 prescaler = 256 → PWM clock = 16 MHz / 256 = 62.5 kHz
 *         - Timer0 prescaler = 1024 → interrupt clock = 16 MHz / 1024 = 15.625 kHz
 */
void PWM_Start()
{
   TCCR1B |= (1 << CS12); 			 			 /*Start PWM signal generation*/
   TCCR0B = TCCR0B | (1 << CS02) | (1 << CS00);  /*Start the synchronisation Timer*/  
}

	
/**
 * @brief  Convert raw 8-bit joystick values to servo pulse widths via linear interpolation.
 *         The joystick is a potentiometer that outputs:
 *           - 0 V at its lower extreme  → ADC = 0
 *           - 2.5 V at center             → ADC = 127
 *           - 5 V at its upper extreme   → ADC = 255
 *         These ADC values ([0…255]) are mapped to the servo’s OCR1x range ([MIN…MAX])
 *         using the precomputed coefficients `a` and `b` (see const.h):
 *           pulse_width = (a + b × ADC_value) / 100
 *
 * @param  joystick           Union that contains the digitally converted value of joystick X/Y axis (0–255, where 127=center)
 */
void Convert_Value_PWM(JoystickData joystick)
{
    servos.sA = (A_OFFSET + B_SLOPE * joystick.x_axis) / 100;   /* Linear interpolation for servo A */
    servos.sB = (A_OFFSET + B_SLOPE * joystick.y_axis) / 100;   /* Linear interpolation for servo B */
}
