/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * @file mpu6050.c														   *
 * @brief																   *
 * Handles initialization and data acquisition from the MPU6050 IMU.       *
 * Provides functions to read accelerometer and gyroscope data, scales raw *
 * sensor values using sensitivity factors, manages FIFO, and triggers     *
 * data-ready interrupts.								                   *
 *																		   *
 *  Pins used:															   *
 *   PA4																   *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32f411xe.h"
#include "const.h"
#include "i2c.h"
#include "delay.h"
#include <stdint.h>

static uint16_t		   	   fifo_count;
static volatile BaseType_t pxHigherPriorityTaskWoken;
extern SemaphoreHandle_t   MPU_Semaphore;

/**
 * @brief Initializes the MPU6050 sensor and configures GPIO, I2C, FIFO,
 *        interrupts, sample rate, DLPF, and full-scale ranges.
 */
void init_mpu6050()
{
    /* Disable global interrupts during configuration */
	__disable_irq();

    /* Enable SYSCFG clock for EXTI configuration */
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    /* Configure PA4 as input with pull-down for MPU INT */
	GPIOA->MODER = (GPIOA->MODER &~GPIO_MODER_MODER4) | (0 << GPIO_MODER_MODER4_Pos);
	GPIOA->PUPDR = (GPIOA->PUPDR &~GPIO_PUPDR_PUPD4) | (2 << GPIO_PUPDR_PUPD4_Pos);

    /* Map EXTI4 to PA4 */
	SYSCFG->EXTICR[1] = (SYSCFG->EXTICR[1] &~SYSCFG_EXTICR2_EXTI4)  | (0 << SYSCFG_EXTICR2_EXTI4_Pos);

    /* Unmask EXTI4 and set rising edge trigger */
	EXTI->IMR |= EXTI_IMR_MR4;
	EXTI->RTSR |= EXTI_RTSR_TR4;

    /* Enable EXTI4 interrupt in NVIC */
	NVIC_EnableIRQ(EXTI4_IRQn);

    /* Re-enable global interrupts */
	__enable_irq();

    /* Reset MPU6050 device */
	mpu_write(PWR_MGMT_1_R,(uint8_t[]){0x80});
	delay_ms(100);

    /* Reset accelerometer and gyroscope signal paths */
	mpu_write(SIGNAL_PATH_RESET_R,(uint8_t[]){0x06});
	delay_ms(100);

    /* Wake up device, use gyro X PLL as clock, disable temperature sensor */
	mpu_write(PWR_MGMT_1_R,(uint8_t[]){0x09});

    /* Configure digital low-pass filter and sample rate */
	/* If Gyro Output Rate = 1 kHz (filter enabled), Sample Rate = Gyro Output Rate / (1 + SMPLRT_DIV_R) = 1 kHz */
	mpu_write(CONFIG_R,(uint8_t[]){DLPF_CFG});
	mpu_write(SMPLRT_DIV_R,(uint8_t[]){SMPLRT_DIV});

    /* Configure full-scale ranges */
	mpu_write(ACCEL_CONFIG_R,(uint8_t[]){AFS_SEL});
	mpu_write(GYRO_CONFIG_R,(uint8_t[]){FS_SEL});

    /* Reset FIFO to remove old data */
	mpu_write(USER_CTRL_R,(uint8_t[]){0x04});
	delay_ms(1);

    /* Enable FIFO for desired sensor bytes */
	mpu_write(FIFO_EN_R,(uint8_t[]){0x78});
	mpu_write(USER_CTRL_R,(uint8_t[]){0x40});

    /* Configure INT pin as active-high, push-pull, and enable DATA_RDY interrupt */
	mpu_write(INT_PIN_CFG_R,(uint8_t[]){0x00});
	mpu_write(INT_ENABLE_R,(uint8_t[]){DATA_RDY_EN});
}

/**
 * @brief Reads a single byte from the MPU6050 register.
 * @param reg_addr Register address to read from
 * @param reg Pointer to store the read value
 */
void mpu_read (uint8_t reg_addr,uint8_t* reg)
{
    /* Perform an I2C read action: read 1 byte from the specified register */
	i2c1_perform_action(READ, MPU_I2C_ADDR, reg_addr, 1, reg);
}

/**
 * @brief Writes a single byte to the MPU6050 register.
 * @param reg_addr Register address to write to
 * @param value Pointer to the value to write
 */
void mpu_write (uint8_t reg_addr, uint8_t* value)
{
    /* Perform an I2C write action: write 1 byte to the specified register */
	i2c1_perform_action(WRITE, MPU_I2C_ADDR, reg_addr, 1, value);
}

/**
 * @brief Reads accelerometer and gyroscope data from MPU6050 FIFO.
 *        Applies scale factors to convert raw data to physical units.
 * @param buf Float array of size 6 to store acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z
 * @return 1 if successful, 0 if FIFO overflow occurred
 */
Status_t mpu_read_acc_gyr (float buf[6])
{
	uint8_t raw_buf[NUM_BYTES];
	uint8_t count_buf[2];

	/* Wait until valid data is available (does not mean it is already in the FIFO) */
	xSemaphoreTake(MPU_Semaphore, portMAX_DELAY);

    /* Clear the DATA_RDY_INT flag */
	uint8_t temp;
	mpu_read(INT_STATUS_R,&temp);

    /* Check for FIFO overflow */
	if(temp & FIFO_OFLOW_INT)
	{
        /* Disable FIFO */
		mpu_write(USER_CTRL_R, (uint8_t[]){0x00});
		vTaskDelay(pdMS_TO_TICKS(1));

        /* Reset and re-enable FIFO */
		mpu_write(USER_CTRL_R,(uint8_t[]){0x04});
		vTaskDelay(pdMS_TO_TICKS(1));
		mpu_write(USER_CTRL_R,(uint8_t[]){0x40});

		return STATUS_NOT_OK; /* Overflow occurred  */
	}

    /* Wait until FIFO contains at least NUM_BYTES */
	while(1)
	{
		i2c1_perform_action(READ, MPU_I2C_ADDR, FIFO_COUNT_H_R, 2U, count_buf);
		fifo_count = ((uint16_t)count_buf[0] << 8U) | (uint16_t)count_buf[1];
		if(fifo_count >= NUM_BYTES) break;
		/* Small delay before reading FIFO again */
		delay_us(FIFO_POLL_US);
	}
    /* Read accelerometer and gyroscope data from FIFO */
	i2c1_perform_action(READ, MPU_I2C_ADDR, FIFO_R_W_R, NUM_BYTES, raw_buf);

    /* Convert raw data to float using sensitivity scale factors */
	float scale_factor;
	for(uint8_t i=0,j = 0;i<NUM_BYTES;i+=2, j++)
	{
		/* First half of buffer is ACC, second half is GYR */
		scale_factor = (i<(NUM_BYTES/2U)) ? ACC_SCALE_FACTOR : GYR_SCALE_FACTOR;

		/* Combine MSB and LSB into int16_t */
		/* The first byte in the buff is the MSB and the second the LSB */
		int16_t val = (int16_t)(((uint16_t)raw_buf[i] << 8U) | (uint16_t)raw_buf[i+1]);

        /* Apply scaling to get physical units */
		buf[j] = (float)val/(scale_factor);
	}
	return STATUS_OK;
}


/**
 * @brief EXTI4 interrupt handler triggered by MPU6050 data-ready signal.
 *        Gives a semaphore to notify tasks that new IMU data is available.
 */
void EXTI4_IRQHandler()
{
    /* Clear EXTI4 pending bit */
	EXTI->PR = EXTI_PR_PR4;

    /* Give semaphore to signal new MPU data available */
	pxHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(MPU_Semaphore, &pxHigherPriorityTaskWoken);

    /* Request context switch if a higher priority task was woken */
	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

