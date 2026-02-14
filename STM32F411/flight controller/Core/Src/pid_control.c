/*  * * * * * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * * * * * * *
 * @file pid_control.c															 *
 * @brief																	     *
 * Implements PID control for the drone's attitude.							     *
 * Provides functions to compute PID errors, normalize angles to avoid wrapping, *
 * and calculate PID outputs with proportional, integral, and derivative terms.  *
 *  * * * * * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * * * * * * */

#include "stm32f411xe.h"
#include "const.h"
#include "stdint.h"
#include "util.h"
#include "dron_structs.h"

__STATIC_INLINE void normalize_error(float* error);

/**
 * @brief Normalizes an angle error to the range (-180, 180] degrees to avoid wrapping issues.
 * @param error Pointer to the angle error to normalize
 */
__STATIC_INLINE void normalize_error(float* error)
{
    /* If error exceeds +180°, wrap it to negative equivalent */
	if(*error>180.0f) 			*error -= 360.0f;
    /* If error is ≤ -180°, wrap it to positive equivalent 	  */
	else if(*error<=-180.0f) 	*error += 360.0f;
}

/**
 * @brief Computes the PID error between the desired setpoint and current measured attitude.
 *        Normalizes roll and pitch errors to prevent discontinuities at ±180°.
 * @param setpoint Desired attitude
 * @param measure Measured attitude
 * @param error Pointer to store computed errors
 */void computePIDError(Attitude_t setpoint, Attitude_t measure, Attitude_t* error)
{
	/* Calculate raw errors for pitch, roll, and yaw rate */
	error->pitch = setpoint.pitch - measure.pitch;
	error->roll = setpoint.roll - measure.roll;
	error->yaw_rate = setpoint.yaw_rate - measure.yaw_rate;

    /* Normalize roll and pitch errors to avoid angle wrapping issues */
	normalize_error(&error->pitch);
	normalize_error(&error->roll);
}

/**
 * @brief Computes the PID controller output for a given error.
 *        Applies proportional, integral, and derivative calculations.
 *        Constrains output within allowed limits for ESCs.
 * @param error Current error value
 * @param dt Time elapsed since last update (seconds)
 * @param pid_params Pointer to PID controller parameters and state
 * @return PID output
 */
float compute_PID(float error, float dt, PID_Controller_t* pid_params)
{
    /* Proportional term */
	float proportional = pid_params->kp * error;

    /* Integrate error over time for the integral term */
	pid_params->integral += error * dt;
	float integral = pid_params->ki * pid_params->integral;

    /* Derivative term based on change in error over time */
	float derivative = pid_params->kd * (error - pid_params->prev_error) / dt;

    /* Store current error for next derivative calculation */
	pid_params->prev_error = error;

    /* Combine P, I, and D terms */
    float output = proportional + integral + derivative;

    /* Constrain output to limits (e.g., 125–250 µs for ESC PWM OneShot125) */
    constrain(&output, pid_params->output_min, pid_params->output_max);

    return output;
}




