/*
 * Wireless Dual-Servo Controller
 * (c) 2025 Hugo Roldán López
 * This file is part of the project licensed under the MIT License.
 * See the LICENSE file in the root of the repository for full license text.
 */
 
 
#ifndef COMMON_H
#define COMMON_H

#include "const.h"
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>

/* Definition of the union that will contain the joystick data */
typedef union {
    struct {
        uint8_t x1_axis;
        uint8_t y1_axis;
        uint8_t x2_axis;
        uint8_t y2_axis;
    };
    uint8_t axis[NUM_ELEMENTS];
} JoystickData;

#endif
