/*
 * Wireless Dual-Servo Controller
 * (c) 2025 Hugo Roldán López
 * This file is part of the project licensed under the MIT License.
 * See the LICENSE file in the root of the repository for full license text.
 */
 
 
/***********************************************************************************************
 * const.h
 *
 * Brief Description:
 *   This file contains project-wide constant definitions grouped by module/area for
 *   clarity and maintainability. Constants include pin mappings, timing parameters,
 *   communication commands, and configuration values.
 ***********************************************************************************************/

#ifndef CONST_H
#define CONST_H


#define TRANSMITTER 0
#define RECEIVER 1

/*==========================================================================================*/
/*                                  SPI INTERFACE                                           */
/*==========================================================================================*/

#define MOSI_PIN        PB3    /* Master Out, Slave In */
#define MISO_PIN        PB4    /* Master In, Slave Out  */
#define CLK_PIN         PB5    /* SPI clock             */

#define DD_MOSI         DDB3   /* Data direction: MOSI  */
#define DD_MISO         DDB4   /* Data direction: MISO  */
#define DD_CLK          DDB5   /* Data direction: SCK   */

#define DD_SS_SLAVE     DDB2   /* Slave Select for SPI slave mode */
#define SS_PIN          PD7    /* Slave Select input from NRF24L01 */
#define DD_SS           DDD7   /* Data direction: SS pin */

#define NUM_ELEMENTS    (4U)   /* Number of motors to control */


/*==========================================================================================*/
/*                                  ADC SETTINGS                                            */
/*==========================================================================================*/

#define F_AUTO_TRIGGER       (2U)                /* Autotrigger multiplier (increments per µs) */
#define AUTO_TRIGGER_TIME    (200U)              /* Time between ADC conversions (µs) */
#define AUTO_TRIGGER_PERIOD  (AUTO_TRIGGER_TIME * F_AUTO_TRIGGER)

#define PIN_X1                PC0    /* Joystick X1 ADC channel */
#define PIN_Y1                PC1    /* Joystick Y1 ADC channel */
#define PIN_X2                PC2    /* Joystick X2 ADC channel */
#define PIN_Y2                PC3    /* Joystick Y2 ADC channel */

#define ADMUX_MUX             (15U)  /* ADMUX multiplexer mask */


/*==========================================================================================*/
/*                                NRF24L01+ RADIO SETTINGS                                  */
/*==========================================================================================*/

#define ACTIVATION_KEY     (0x73)   /* Payload for ACTIVATE command */
#define ADDRESS_WIDTH      (5U)     /* Address width in bytes */

#define CE_PIN             PB0      /* Chip Enable for NRF24L01 */
#define DD_CE              DDB0     /* Data direction: CE pin */
#define INT0_PIN           PD2      /* External interrupt 0 input */
#define DD_INT0            DDD2     /* Data direction: INT0 pin (PD2)*/
#define DD_INT1            DDD3     /* Data direction: INT1 pin (PD3)*/

#define RX_DS 6			    /* RX_DS flag in STATUS register */


/*==========================================================================================*/
/*                               NRF24L01+ COMMAND CODES                                    */
/*==========================================================================================*/

/* Write Registers */
#define W_CONFIG            (0x20)
#define W_EN_AA             (0x21)
#define W_EN_RXADDR         (0x22)
#define W_SETUP_AW          (0x23)
#define W_SETUP_RETR        (0x24)
#define W_RF_CH             (0x25)
#define W_RF_SETUP          (0x26)
#define W_STATUS            (0x27)
#define W_RX_ADDR_P0        (0x2A)
#define W_TX_ADDR           (0x30)
#define W_RX_PW_P0 	        (0x31) 
#define W_RX_PW_P1          (0x32)
#define W_FEATURE           (0x3D)  /* Activate NO_ACK feature */
#define W_TX_PAYLOAD_NO_ACK (0xB0)  /* Deactivates the AUTOACK (sets NO_ACK flag) in this specific paquet*/
#define W_DYNPD             (0x3C)
#define W_TX_PAYLOAD        (0xA0)
#define W_ACK_PAYLOAD_P0    (0xA8)   /*Write ACK payload in pipe 0 */
#define EN_ACK_PAY          (0x02)   /* Enable Payload with ACK bit */
#define EN_DPL              (0x04)   /* Enable Dynamic Payload bit  */
#define DPL_P0              (0x00)   /* Enable dyn. payload length data pipe 0 bit*/
#define EN_AA_P0            (0x01)

/* Read Registers */
#define R_CONFIG_R          (0x00)
#define R_SETUP_AW_R        (0x03)
#define R_SETUP_RETR_R      (0x04)
#define R_RF_CH_R           (0x05)
#define R_RF_SETUP_R        (0x06)
#define R_STATUS            (0x07)
#define R_RX_PAYLOAD        (0x61)
#define R_TX_ADDR_R         (0x10)


/* Other Commands */
#define NOP                (0xFF)
#define FLUSH_RX           (0xE2)
#define FLUSH_TX           (0xE1)
#define ACTIVATE           (0x50)    /* Follow with ACTIVATION_KEY */

#define NO_ACK             (1U)
#define ACK                (0U)

/*==========================================================================================*/
/*                                 USART SETTINGS                                           */
/*==========================================================================================*/

#define FOSC               (16000000UL)  /* MCU clock frequency */
#define BAUD               (9600U)       /* UART baud rate */
#define UBRR               ((FOSC / 16U / BAUD) - 1U)


/*==========================================================================================*/
/*                                 TELEMETRY                                                */
/*==========================================================================================*/

#define FOSC_MHZ             (16U)
#define TIMER0_PRESCALER     (1024U)
#define NUM_CYCLES           (10U) 
#define CYCLE_TIME_US        (10000U)
#define PACKET_ACK_PERIOD_US (NUM_CYCLES * CYCLE_TIME_US)

#define OCR0A_TICKS          ((CYCLE_TIME_US * FOSC_MHZ)/TIMER0_PRESCALER)) 
#define TELEM_FRAME_SIZE     (4U)
#define SEND_ACK_PERIOD_MS   (100U)


/*==========================================================================================*/
/*                                 CIRCULAR QUEUE                                           */
/*==========================================================================================*/

#define CIRCULAR_QUEUE_SIZE_RX     (TELEM_FRAME_SIZE * 6U)
#define CIRCULAR_QUEUE_SIZE_TX     (NUM_ELEMENTS * 6U)


/*==========================================================================================*/
/*                                 COOPERATIVE SCHEDULER                                    */
/*==========================================================================================*/

#define SCHEDULER_PERIOD            (250U)
#define MAX_TASKS                   (8U)
#define SEND_DATA_TASK_PERIOD_MS    (1U)
#define RECEIVE_DATA_TASK_PERIOD_MS (1U)
#define TX_TELEMETRY_TASK_PERIOD_MS (100U)
#define RX_TELEMETRY_TASK_PERIOD_MS (100U)

#endif
