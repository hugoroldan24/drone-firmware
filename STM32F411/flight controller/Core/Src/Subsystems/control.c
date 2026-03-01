/*  * * * * * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * * * * * * *
 * @file pid_control.c															 *
 * @brief																	     *
 * Implements PID control for the drone's attitude.							     *
 * Provides functions to compute PID errors, normalize angles to avoid wrapping, *
 * and calculate PID outputs with proportional, integral, and derivative terms.  *
 *  * * * * * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * * * * * * */

#include "stm32f411xe.h"
#include "config.h"
#include "stdint.h"
#include "util.h"
#include "dron_structs.h"
#include "pwm.h"

extern QueueHandle_t  flight_data_queue_ID, pid_error_queue_ID;
extern TaskHandle_t   IMU_ID, RC_RX_ID, PID_ERROR_ID;


__STATIC_INLINE void normalize_error(float* error);
 

/**
 * @brief Task computing PID error for roll, pitch, and yaw_rate.
 *
 * Receives setpoints and IMU data from the flight data queue, computes
 * the PID error, and sends it to the PID error queue for the control task.
 */
void xHandlePIDErrorTask(void* parameters)
{
	DroneState_t system_state;
	while(1)
	{
		system_state = get_drone_state();
		switch(system_state)
		{
			case STATE_BOOT:
			    /* Suspend self until activated */
	           vTaskSuspend(NULL);
			break;

			case STATE_STANDBY:
			/* The task should not been resumed in this state */
			break;

			case STATE_FLIGHT:
			break;

			case STATE_LANDING:
			break;
		}
	}
	/* Suspend self until activated */
	vTaskSuspend(NULL);

	FlightMessage_t rx_setpoint, rx_imu, pid_error;

	while(1){
        /* Receive IMU data from queue */
		xQueueReceive(flight_data_queue_ID, &rx_imu, portMAX_DELAY);

        /* Receive setpoint from queue */
		xQueueReceive(flight_data_queue_ID, &rx_setpoint, portMAX_DELAY);

        /* Preserve throttle for control task */
		pid_error.throttle = rx_setpoint.throttle;

        /* Compute PID error */
		computePIDError(rx_setpoint.attitude, rx_imu.attitude, &pid_error.attitude);

        /* Send PID error to control task */
		xQueueSend(pid_error_queue_ID, (void *)&pid_error,portMAX_DELAY);
	}
}


/**
 * @brief Task for PID control and motor mixing.
 *
 * Receives PID errors, computes PID outputs for roll, pitch, and yaw_rate,
 * mixes outputs into motor commands, and updates PWM signals through
 * the motor mixer.
 */
void xHandleControlTask(void* parameters)
{
	DroneState_t system_state;
	PWM_Outputs_t ccr;
	FlightMessage_t  pid_error;
	PID_Controller_t roll_pid_params ;
	PID_Controller_t pitch_pid_params;
	PID_Controller_t yaw_pid_params;
	PID_Outputs_t    output;

	uint32_t lastcall = 0;
	float dt;
	const TickType_t xFrequency = pdMS_TO_TICKS(3U);

	set_pid_params(&roll_pid_params, &pitch_pid_params, &yaw_pid_params);

	while(1)
	{
		system_state = get_drone_state();
		switch(system_state)
		{
			case STATE_BOOT:
	 		   /* Suspend self until activated */
	           vTaskSuspend(NULL);
			break;

			case STATE_STANDBY: 
			/* The task should not been resumed in this state */
			break;

			case STATE_FLIGHT:
			break;

			case STATE_LANDING:
			break;
		}
	}


    xLastWakeTime = xTaskGetTickCount();

	lastcall = get_time_now_us();
	while(1)
	{
		xQueueReceive(pid_error_queue,&pid_error,portMAX_DELAY);

		dt = get_time_elapsed(&lastcall);

		output.roll 	= compute_PID(pid_error.attitude.roll,     dt, &roll_pid_params);
		output.pitch 	= compute_PID(pid_error.attitude.pitch,    dt, &pitch_pid_params);
		output.yaw_rate = compute_PID(pid_error.attitude.yaw_rate, dt, &yaw_pid_params);

		motor_mixer(output,pid_error.throttle,&ccr); 
		
		/* Make sure that all the other tasks except telemetry ones are blocked
		so that in this delay time they can execute (or IDLE to save power)*/
		vTaskDelayUntil(&xLastWakeTime,xFrequency); 

		update_PWM_Outputs(TIM3,ccr);

		/* Signal IMU task to start a new cycle */
		xTaskNotifyGive(IMU_ID);
	}
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
 * @brief Static function that initialices the pid parameters structs to the desired values
 * defined in "config.h"        
 */
static void set_pid_params(PID_Controller_t *roll, PID_Controller_t *pitch, PID_Controller_t *yaw)
{
   roll->kp = ROLL_KP;
   roll->ki = ROLL_KI;
   roll->kd = ROLL_KD;
   roll->output_min = ROLL_OUTPUT_MIN;
   roll->output_max = ROLL_OUTPUT_MAX;

   pitch->kp = PITCH_KP;
   pitch->ki = PITCH_KI;
   pitch->kd = PITCH_KD;
   pitch->output_min = PITCH_OUTPUT_MIN;
   pitch->output_max = PITCH_OUTPUT_MAX;

   yaw->kp = YAW_KP;
   yaw->ki = YAW_KI;
   yaw->kd = YAW_KD;
   yaw->output_min = YAW_OUTPUT_MIN;
   yaw->output_max = YAW_OUTPUT_MAX;
}


