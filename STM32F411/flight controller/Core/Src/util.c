/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * @file util.c																		 *
 * @brief Utility functions used throughout the project.							 *
 *																					 *
 * This file contains general-purpose helper functions, such as time measurement and *
 * value constraining, which are used by various modules.							 *
 * These functions provide reusable functionality to simplify the main control code. *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "stm32f411xe.h"
#include "const.h"

/**
 * @brief Initialize Timer 2 as a free-running microsecond timer.
 *
 * Sets up Timer 2 with a prescaler so that each tick equals 1 µs.
 * Used to measure elapsed time between function calls.
 */
void dt_timer_init()
{
	/* Enable clock access to Timer 2 */
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Set prescaler so that 1 tick = 1 µs (APB1 timer clock = 100 MHz) */
	TIM2->PSC = 2*APB1_CLK_MHZ - 1;  // 100 MHz / (99 +1) = 1 MHz -> 1 tick = 1us

	/* Configure upcounting mode */
	TIM2->CR1 &= ~TIM_CR1_DIR;

	/* Start the timer */
	TIM2->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief Get the current timer value in microseconds.
 *
 * Reads the current value of the free-running Timer 2.
 * @return Current timer value (µs).
 */
uint32_t get_time_now_us()
{
	/* Read the current value of the free-running timer */
	uint32_t now = TIM2->CNT;
	return now;
}

/**
 * @brief Calculate the elapsed time since the last call.
 *
 * Computes the difference between the current timer value and the last recorded value,
 * updates the last call time, and converts it to seconds.
 * @param lastcall Pointer to the variable storing the last call timestamp.
 * @return Elapsed time in seconds.
 */
float get_time_elapsed(uint32_t* lastcall)
{
	/* Read current timer count */
	uint32_t now = TIM2->CNT;

	/* Compute difference with previous call, taking wrap-around into account */
	uint32_t diff = now - *lastcall;

	/* Update last call timestamp */
	*lastcall = now;

	/* Convert microseconds to seconds */
	float dt = US_TO_S(diff);
	return dt;
}

/**
 * @brief Constrain a float value between a lower and upper bound.
 *
 * Ensures the value does not exceed the specified limits.
 * @param value Pointer to the float value to constrain.
 * @param low Minimum allowed value.
 * @param high Maximum allowed value.
 */
void constrain(float* value, float low, float high)
{
	if(*value<low)
	{
		*value = low;
	}
	else if(*value>high)
	{
		*value = high;
	}
}



