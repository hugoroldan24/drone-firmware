/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * @file delay_timer.c																		   *
 * @brief																					   *
 * Implements precise delays using Timer 5. Provides microsecond and millisecond delays,	   *
 * internal tick-based wait, timer initialization, and interrupt handling for delay completion.*
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "stm32f411xe.h"
#include <stdint.h>
#include "const.h"

__STATIC_INLINE void _delay_ticks(uint32_t ticks);
static volatile uint8_t flag;

/**
 * @brief Initializes Timer 5 for precise delays.
 *        Configures prescaler, counting mode, auto-reload, one-pulse mode,
 *        and enables the update interrupt in NVIC.
 */
void init_delay_timer()
{
    /* Enable clock for Timer 5 */
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;

	/* Set prescaler, since the prescaler for APB1 bus is 2, the timer clock is twice the bus freq. so it is 100 MHz */
	TIM5->PSC = 2*APB1_CLK_MHZ - 1;  // 100 MHz / (99 +1) = 1 MHz -> 1 tick = 1us

	/* Upcounting mode */
	TIM5->CR1 &= ~TIM_CR1_DIR;

    /* Enable buffered auto-reload */
	TIM5->CR1 |= TIM_CR1_ARPE;

    /* One Pulse Mode: timer stops at ARR */
	TIM5->CR1 |= TIM_CR1_OPM;

    /* Only overflow triggers interrupt */
	TIM5->CR1 |= TIM_CR1_URS;

    /* Reset counter */
	TIM5->CNT = 0;

    /* Clear any pending interrupt */
	TIM5->SR &= ~TIM_SR_UIF;

    /* Enable update interrupt */
	TIM5->DIER |= TIM_DIER_UIE;

    /* Enable Timer 5 IRQ in NVIC */
	NVIC_EnableIRQ(TIM5_IRQn);
}

/**
 * @brief Internal tick-based delay.
 *        Blocks until the timer reaches the specified tick count.
 * @param ticks Number of timer ticks to wait.
 */
__STATIC_INLINE void _delay_ticks(uint32_t ticks)
{
	flag = 0; /* Reset completion flag */

    /* Set auto-reload value (timer stops at ARR) */
	TIM5->ARR = ticks - 1;

    /* Force update to restart counter from 0 */
	TIM5->EGR |= TIM_EGR_UG;

    /* Start timer counting */
	TIM5->CR1 |= TIM_CR1_CEN;

    /* Wait until timer interrupt sets the flag */
	while(!flag);
}

/**
 * @brief Delay for a specified number of milliseconds.
 * @param ms Time to delay in milliseconds.
 */
void delay_ms(uint32_t ms){ _delay_ticks(1000*ms);}

/**
 * @brief Delay for a specified number of microseconds.
 * @param us Time to delay in microseconds.
 */
void delay_us(uint32_t us){ _delay_ticks(us);}

/**
 * @brief Timer 5 interrupt handler.
 *        Clears the update interrupt flag and sets the delay completion flag.
 */
void TIM5_IRQHandler()
{
	if (TIM5->SR & TIM_SR_UIF)
	{
		TIM5->SR &= ~TIM_SR_UIF; /* Clear update interrupt flag */
		flag =  1;				 /* Signal delay completion */
	}

}



