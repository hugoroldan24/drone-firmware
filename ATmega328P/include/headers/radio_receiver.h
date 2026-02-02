/*
 * Wireless Dual-Servo Controller
 * (c) 2025 Hugo Roldán López
 * This file is part of the project licensed under the MIT License.
 * See the LICENSE file in the root of the repository for full license text.
 */
 
 
#ifndef RADIO_H
#define RADIO_H


#include "common.h"

void RF_Receiver_Init(void);
void Radio_Listen(void);
void writeRegister(uint8_t writeCommand,uint8_t conf);
void sendCommand(uint8_t command);
void writeAddress(uint8_t pipe,uint8_t *addr,uint8_t size);
uint8_t readRegister(uint8_t reg);
void get_Received_Data(JoystickData *joystick);
void send_ACK_Payload(uint8_t *payload, uint32_t len);

extern volatile int8_t availableData;   /*This flag is activated when the RF module places data in the RX_FIFO register*/

#endif
