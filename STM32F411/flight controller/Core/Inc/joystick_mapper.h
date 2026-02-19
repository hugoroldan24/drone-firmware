
#ifndef JOYSTICK_MAPPER_H_
#define JOYSTICK_MAPPER_H_

#include "dron_structs.h"

uint8_t get_joystick_data(UserControl_t* input);
void map_joystick_to_setpoint(UserControl_t input, FlightMessage_t* setpoint);



#endif /* JOYSTICK_MAPPER_H_ */

