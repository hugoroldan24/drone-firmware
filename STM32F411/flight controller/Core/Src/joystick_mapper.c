/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * @file joystick_mapper.c														 *
 * @brief																		 *
 * Reads analog joystick inputs via UART, processes the data, and maps the		 *
 * joystick axes to drone control setpoints: roll, pitch, yaw rate, and throttle.*
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "stm32f411xe.h"
#include "const.h"
#include "uart.h"
#include "dron_structs.h"


/**
 * @brief Reads joystick data from UART into the UserControl structure.
 * @param input Pointer to UserControl_t structure to store joystick axis values
 */
void get_joystick_data(UserControl_t* input)
{
	/* Read joystick bytes from UART via DMA into the axis array.
	       FRAME_LEN-1 excludes protocol-specific start byte and checksum.
	       Order of bytes is important to match channel mapping. */
	uart1_read_dma(input->axis,FRAME_LEN-1);
}


/**
 * @brief Maps the raw joystick axes to drone control setpoints.
 *        Left joystick: Y → throttle, X → yaw rate
 *        Right joystick: Y → pitch, X → roll
 * @param input UserControl_t structure containing raw joystick axes
 * @param setpoint Pointer to FlightMessage_t structure to store mapped setpoints
 */
void map_joystick_to_setpoint(UserControl_t input, FlightMessage_t* setpoint)
{
	setpoint->attitude.roll     =    MAP_JOYSTICK_TO_ANGLE(input.right_x_axis);   /* Map right joystick horizontal (X) to roll angle */
	setpoint->attitude.pitch    = 	 MAP_JOYSTICK_TO_ANGLE(input.right_y_axis);   /* Map right joystick vertical (Y) to pitch angle */
	setpoint->attitude.yaw_rate = 	 MAP_JOYSTICK_TO_YAW(input.left_x_axis);      /* Map left joystick horizontal (X) to yaw rate */
	setpoint->throttle 			=    MAP_JOYSTICK_TO_THROTTLE(input.left_y_axis); /* Map left joystick vertical (Y) to throttle */
}


