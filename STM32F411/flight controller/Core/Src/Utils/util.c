/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * @file util.c																		 *
 * @brief Utility functions used throughout the project.							 *
 *																					 *
 * This file contains general-purpose helper functions, such as time measurement and *
 * value constraining, which are used by various modules.							 *
 * These functions provide reusable functionality to simplify the main control code. *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "stm32f411xe.h"
#include "config.h"

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



