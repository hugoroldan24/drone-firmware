/*
 * Wireless Dual-Servo Controller
 * (c) 2025 Hugo Roldán López
 * This file is part of the project licensed under the MIT License.
 * See the LICENSE file in the root of the repository for full license text.
 */
 
 
#ifndef PWM_H
#define PWM_H

#include "common.h"

/* Definition of the struct that will contain the computed OCR1A value for servo A and servo B */
typedef struct{
   uint16_t sA;
   uint16_t sB;
} Servo;

void PWM_Init(void);
void PWM_Timer0_Init(void);
void PWM_Start(void);
void Convert_Value_PWM(JoystickData joystick);



#endif
