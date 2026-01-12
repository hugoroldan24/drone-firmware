#ifndef PID_CONTROL_H_
#define PID_CONTROL_H_
#include "dron_structs.h"

void  computePIDError(Attitude_t setpoint, Attitude_t measure, Attitude_t* error);
float compute_PID(float error, float dt, PID_Controller_t* pid_params);



#endif /* PID_CONTROL_H_ */

