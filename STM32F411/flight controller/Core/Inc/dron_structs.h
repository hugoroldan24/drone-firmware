/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 * @file dron_structs.h																				 *
 * @brief Data structures used across the FC firmware to improve data organization and readability.  *
 *																									 *
 * This file defines several structs and unions that are used to represent:							 *
 * - The drone's attitude (roll, pitch, yaw rate)													 *
 * - Flight messages containing attitude and throttle information									 *
 * - User input from joysticks																		 *
 * - PWM outputs for motors																			 *
 * - PID controller outputs and configuration														 *
 * - Telemetry data including battery level and pressure											 *	
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef DRON_STRUCTS_H_
#define DRON_STRUCTS_H_

#include <stdint.h>

typedef struct {
    float roll;
    float pitch;
    float yaw_rate; /* Since we won't use magnetometer, if we deal with yaw as an angle, it will end up not working so we will work with the angular speed */
} Attitude_t;

typedef struct {
	Attitude_t  attitude;
	float 		throttle;
	uint8_t		type;
} FlightMessage_t;

typedef union {
	struct{
		uint8_t left_x_axis;
		uint8_t left_y_axis;
		uint8_t right_x_axis;
		uint8_t right_y_axis;
	};
	uint8_t axis[4];

} UserControl_t;

typedef struct {
    uint16_t motor1_pwm;
    uint16_t motor2_pwm;
    uint16_t motor3_pwm;
    uint16_t motor4_pwm;
} PWM_Outputs_t;

typedef struct{
	float roll;
	float pitch;
	float yaw_rate;
} PID_Outputs_t;

typedef struct {
    float kp;           // Proportional gain
    float ki;           // Integral gain
    float kd;           // Derivative gain

    float integral;     // Integral term accumulator
    float prev_error;   // Previous error (for derivative calculation)

    float output_min;   // Minimum output limit (-1)
    float output_max;   // Maximum output limit (1)
} PID_Controller_t;

typedef struct {
	int32_t  preassure;
	uint16_t battery_level;
} Telem_t;

#endif /* DRON_STRUCTS_H_ */


