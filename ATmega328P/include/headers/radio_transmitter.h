/*
 * Wireless Dual-Servo Controller
 * (c) 2025 Hugo Roldán López
 * This file is part of the project licensed under the MIT License.
 * See the LICENSE file in the root of the repository for full license text.
 */
 
 
#ifndef RADIO_H
#define RADIO_H

void RF_Transmitter_Init();
void sendPaquet(JoystickData joystick, uint8_t no_ack);
void writeRegister(uint8_t writeCommand,uint8_t conf);
void sendCommand(uint8_t command);
void writeAddress(uint8_t pipe,uint8_t *addr,uint8_t size);
uint8_t readRegister(uint8_t reg);
void get_Telem_Data(uint8_t* data, uint8_t len);


#endif
