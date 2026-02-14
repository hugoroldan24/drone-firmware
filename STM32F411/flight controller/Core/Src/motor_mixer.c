/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * @file motor_mixer.c															     *
 * @brief																			 *
 * Computes PWM setpoints for the drone motors based on throttle and PID outputs.    *
 * Applies motor mixing logic considering pitch, roll, and yaw rate, and constrains  *
 * the PWM values to valid ranges for the motor controllers.				         *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "const.h"
#include "util.h"
#include "dron_structs.h"

/**
 * @brief Mixes throttle and PID outputs to calculate individual motor PWM values.
 *        Ensures values are constrained within safe PWM limits and stores them
 *        in the PWM_Outputs_t structure for each motor.
 *
 * @param pid_outputs Structure containing PID outputs for pitch, roll, and yaw rate
 * @param throttle Current throttle input from joystick
 * @param ccr Pointer to PWM_Outputs_t to store computed PWM values for each motor
 */
void motor_mixer(PID_Outputs_t pid_outputs, float throttle, PWM_Outputs_t* ccr)
{
	/* Compute preliminary PWM values by combining throttle with PID corrections.
	       Motor mixing logic takes into account the effect of pitch, roll, and yaw on each motor.
	       Adjust signs according to motor orientation/polarity. */
	float pwm1 = (throttle + pid_outputs.pitch + pid_outputs.roll - pid_outputs.yaw_rate);
	float pwm2 = (throttle + pid_outputs.pitch - pid_outputs.roll + pid_outputs.yaw_rate);
	float pwm3 = (throttle - pid_outputs.pitch - pid_outputs.roll - pid_outputs.yaw_rate);
	float pwm4 = (throttle - pid_outputs.pitch + pid_outputs.roll + pid_outputs.yaw_rate);

    /* Constrain each PWM value to safe operational range (125–250 µs) */
	constrain(&pwm1,125U,250U);
	constrain(&pwm2,125U,250U);
	constrain(&pwm3,125U,250U);
	constrain(&pwm4,125U,250U);

    /* Cast to uint16_t and store in output structure for each motor */
	ccr->motor1_pwm =	(uint16_t) pwm1;
	ccr->motor2_pwm =	(uint16_t) pwm2;
	ccr->motor3_pwm =	(uint16_t) pwm3;
	ccr->motor4_pwm =	(uint16_t) pwm4;
    /* At this point, ccr contains the PWM values ready to be loaded into the timers' CCR registers */
}

