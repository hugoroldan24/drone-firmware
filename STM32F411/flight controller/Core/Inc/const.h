/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * @file const.h                                                                             *
 * @brief                                                                                    *
 *   This file contains project-wide constant definitions grouped by module/area for         *
 *   clarity and maintainability. Constants include clock settings, communication commands,  *
 *   peripheral addresses, and configuration parameters.                                     *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */ 

#ifndef CONST_H_
#define CONST_H_

#include <stdint.h>

/*==========================================================================================*/
/*                                  SYSTEM CLOCK                                            */
/*==========================================================================================*/

#define HSE_CLK             (8000000UL)   /* External crystal oscillator frequency */
#define SYS_CLK             (100000000UL) /* System clock frequency after PLL       */

#define APB2_CLK            SYS_CLK      		 /* APB2 bus clock frequency              	   	 */
#define APB2_CLK_MHZ		(APB2_CLK/1000000UL) /* APB1 bus clock frequency in MHz              */
#define APB1_CLK            (SYS_CLK/2U) 		 /* APB1 bus clock frequency           		     */
#define APB1_CLK_MHZ		(APB1_CLK/1000000UL) /* APB1 bus clock frequency in MHz              */
#define APB1_PERIOD_NS		(1000UL/APB1_CLK_MHZ)

/*==========================================================================================*/
/*                                 UART SETTINGS                                            */
/*==========================================================================================*/

#define BAUD             (9600UL)   /* UART baud rate         */
#define UART_BUF_SIZE    (128U)
#define NUM_AXIS	     (4U)		/* 2 joysticks => 4 axis */

#define TELEM_FRAME_SIZE (4U)
#define FRAME_LEN        (5U)

/*==========================================================================================*/
/*                                  I2C SETTINGS                                            */
/*==========================================================================================*/

#define I2C_READ            (1U)         /* I2C read flag                         */
#define I2C_WRITE           (0U)         /* I2C write flag                        */
#define READ                (0U)
#define WRITE               (1U)
#define COMMAND             (2U)
#define MAX_FM              (300U)        /* I2C max frequency margin (ns)       */
#define DESIRED_T			(2500)		  /* I2C desired period in ns (400 kHz)  */
#define DUMMY               (0xFF)    	  /* Dummy byte for SPI/I2C transfers    */

#define I2C_SM_100KHZ      (250U)       /* Value calculated from the CCR[11:0] equation for 100 kHz I2C standard mode */
#define SM_MAX_RISE_TIME   (50U + 1U)   /* Maximum rise time as specified by the I2C standard */

/*==========================================================================================*/
/*                               I2C STATE MACHINE                                          */
/*==========================================================================================*/

typedef enum {
    EV5,
    EV6,
    EV8,
    EV7,
    EV8_2
} I2C_State_t;

/*==========================================================================================*/
/*                            MPU6050 (IMU) REGISTER MAP                                   */
/*==========================================================================================*/

#define MPU_I2C_ADDR            (0x68U)    /* MPU6050 I2C address                 */
#define FIFO_EN_R               (0x23U)    /* FIFO enable register address        */
#define ACCEL_CONFIG_R          (0x1CU)    /* Accelerometer config register       */
#define GYRO_CONFIG_R           (0x1BU)    /* Gyroscope config register           */
#define SMPLRT_DIV_R            (0x19U)    /* Sample rate divider register        */
#define CONFIG_R                (0x1AU)    /* DLPF config register                */
#define INT_ENABLE_R            (0x38U)    /* Interrupt enable register           */
#define INT_STATUS_R            (0x3AU)    /* Interrupt status register           */
#define FIFO_R_W_R              (0x74U)    /* FIFO read/write register            */
#define USER_CTRL_R             (0x6AU)    /* User control register               */
#define PWR_MGMT_1_R            (0x6BU)    /* Power management 1 register         */
#define SIGNAL_PATH_RESET_R		(0x68U)	  /* Signal path reset register			 */
#define INT_PIN_CFG_R           (0x37U)    /* Interrupt Pin / Bypass Enable Configuration register */
#define WHO_AM_I_R              (0x75U)    /* Device identification register, returns 0x68 for MPU6050 */
#define FIFO_COUNT_H_R          (0x72U)    /* High byte of the FIFO sample count (number of bytes in FIFO) */
#define FIFO_COUNT_L_R          (0x73U)    /* Low byte of the FIFO sample count */



#define AFS_SEL                 (1U<<3U)   	/* Accelerometer full-scale select (4g)*/
#define FIFO_EN_R_CONFIG        (0x78U)    	/* FIFO enable config for gyro/accel    */
#define DEVICE_RESET            (1U<<7U)   	/* Device reset bit in PWR_MGMT_1      */
#define TEMP_DIS                (1U<<3U)   	/* Disable temperature sensor bit      */
#define FIFO_RESET              (0x44U)     /* Reset FIFO command                  */
#define DATA_RDY_EN             (1U<<0U)   	/* Data ready enable bit               */
#define SMPLRT_DIV              (0x00U)     /* Sample rate divider value           */
#define DLPF_CFG                (1U<<1U)    /* Digital low-pass filter configuration*/
#define FS_SEL                  (0x00U)     /* Gyroscope full-scale select (±250°/s)*/
#define FIFO_OFLOW_INT          (1U<<4U)    /* Bit in the INT_STATUS register indicating a FIFO overflow */
#define	ACC_SCALE_FACTOR	    (8192.0f)   /* 4 g */
#define GYR_SCALE_FACTOR		(131.0f)	/* +-250 º/s */
#define FIFO_POLL_US            (200U)      /* Microseconds to wait before checking that the FIFO has the expected number of bytes */

/*==========================================================================================*/
/*                             MS5611 BAROMETER CONSTANTS                                   */
/*==========================================================================================*/

#define MS5611_I2C_ADDR         (0x77U)      /* MS5611 sensor I2C address           */

#define PROM_C1_SENS_ADDR       (0xA2U)      /* MS5611 PROM coefficient C1 address  */
#define PROM_C2_OFF_ADDR        (0xA4U)      /* C2 PROM addr: offset                */
#define PROM_C3_TCS_ADDR        (0xA6U)      /* C3 PROM addr: TCS                   */
#define PROM_C4_TCO_ADDR        (0xA8U)      /* C4 PROM addr: TCO                   */
#define PROM_C5_TREF_ADDR       (0xAAU)      /* C5 PROM addr: reference temp        */
#define PROM_C6_TEMPSENS_ADDR   (0xACU)      /* C6 PROM addr: temp sensitivity      */

#define RESET                   (0x1EU)      /* MS5611 reset command                */
#define CMD_CONVERT_D1_OSR_4096 (0x48U)      /* Pressure conversion command         */
#define CMD_CONVERT_D2_OSR_4096 (0x58U)      /* Temperature conversion command      */
#define ADC_READ                (0x00U)      /* ADC read command                    */

#define NUM_BYTES               (12U)       /* Number of bytes to read from FIFO   */

/*==========================================================================================*/
/*                               PWM				                                        */
/*==========================================================================================*/

#define ESC_PWM_PERIOD 		   (250U)			/* OneShot has not a fixed period, we set it to the maximum period length */
#define IDLE_PULSE			   (125U)			/* 1 ms wide pulse will make the motors be in idle state */
#define UPDATE_REG_PERIOD	   (4000U)			/* Every 4 ms we will update the CCR registes 			*/
#define PWM_MIN                (125U)          /* Minimum PWM pulse width */
#define PWM_MAX                (250U)          /* Maximum PWM pulse width */

/*==========================================================================================*/
/*                               FILTER				                                        */
/*==========================================================================================*/

#define RAD_TO_DEG				(180.0f / 3.14159265359f)
#define ALPHA					(0.98f)		
#define US_TO_S(us) 		    ((float)(us) / 1e6f)

/*==========================================================================================*/
/*                               PID PARAMETERS				                                */
/*==========================================================================================*/

/* To be completed */
#define ROLL_KP				0
#define ROLL_KI				0
#define ROLL_KD				0
#define ROLL_OUTPUT_MIN		0
#define ROLL_OUTPUT_MAX		0

#define PITCH_KP			0
#define PITCH_KI			0
#define PITCH_KD			0
#define PITCH_OUTPUT_MIN	0
#define PITCH_OUTPUT_MAX	0

#define YAW_KP				0
#define YAW_KI				0
#define YAW_KD				0
#define YAW_OUTPUT_MIN		0
#define YAW_OUTPUT_MAX		0


/*==========================================================================================*/
/*                               JOYSTICK MAPPER				                            */
/*==========================================================================================*/

#define JOY_MIN         (0U)
#define JOY_MAX         (255U)
#define JOY_MID         (127U) 

/* To be completed */
#define DESCENT_THROTTLE  0 
#define HOVER_THROTTLE    0
#define NO_THROTTLE       0

#define ANGLE_MAX_DEG   (30.0f)   
#define YAW_RATE_MAX    (100.0f)  

#define ANGLE_SLOPE			((float)(2*ANGLE_MAX_DEG)/JOY_MAX)
#define ANGLE_OFFSET		(-ANGLE_MAX_DEG)

#define YAW_SLOPE			((float)(2*YAW_RATE_MAX)/JOY_MAX)
#define YAW_OFFSET			(-YAW_RATE_MAX)

#define THROTTLE_OFFSET		(PWM_MIN + PID_MARGIN)
#define THROTTLE_SLOPE		((float)(PWM_MAX - PID_MARGIN)- THROTTLE_OFFSET)

#define PID_MARGIN          (200U)    /* Margin left in the throttle for PID correction (may depend on the PID maximum output) */

#define MAP_JOYSTICK_TO_ANGLE(joy)		((float)(ANGLE_SLOPE*joy) + ANGLE_OFFSET)		        /* -30 to +30 deg	  */
#define MAP_JOYSTICK_TO_YAW(joy)		((float)(YAW_SLOPE*joy) + YAW_OFFSET)			        /* -100 to +100 deg/s */
#define MAP_JOYSTICK_TO_THROTTLE(joy)	(((float)joy/JOY_MAX)*THROTTLE_SLOPE + THROTTLE_OFFSET) /* 0.0 to 1.0		  */

/*==========================================================================================*/
/*                              BATTERY	LECTURE					                            */
/*==========================================================================================*/

#define BATTERY_PERIOD  (200000UL) /* Period in us where we will read the battery voltage */

#define ADC_MAX_VALUE   (4095.0f)
#define VREF            (3.3f)
#define DIVIDER_FACTOR  (3.7f)   /* Voltage divider used to scale down the battery voltage so it can be read by the MCU */
                                 /* (27kOhm + 10kOhm) / 10 k0hm = 3.7 */

#define BATTERY_MAX_V  (12.6f)
#define BATTERY_SAFE_V (10.8f) /* When we read this voltage, it means it's time to land */
#define BATTERY_MIN_V  (9.9f) /* When this volatge is reached, the dron will enter in security mode */

/*==========================================================================================*/
/*                              FreeRTOS PARAMETERS				                            */
/*==========================================================================================*/
#define STACK_SIZE_WORDS           (256U)  /* 256 words * 4 bytes = 1024 bytes = 1 KB Stack for each task */
#define TELEM_QUEUE_NUM_ELEM       (6U)
#define FLIGHT_DATA_QUEUE_NUM_ELEM (6U)
#define PID_ERROR_QUEUE_NUM_ELEM   (10U)

#endif /* CONST_H_ */




