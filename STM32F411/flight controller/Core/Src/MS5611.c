/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * @file MS5611.c																     *
 * @brief																		     *
 * Initializes and reads data from the MS5611 barometric pressure sensor.		     *
 * Reads calibration coefficients from PROM, acquires raw temperature and pressure,  *
 * and calculates compensated pressure values using the sensor's formula.		     *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <stdint.h>
#include "spi.h"
#include "const.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"

static void ms5611_read_calibration(void);       // lee y almacena C1–C6
static int32_t ms5611_calc_pressure(uint32_t D1, uint32_t D2);
static uint16_t get_prom_coeff(uint8_t maddr);
static uint32_t get_raw_data(uint8_t command);
static uint16_t C1, C2, C3, C4, C5;

/**
 * @brief Initializes the MS5611 sensor.
 *        Performs a reset and reads calibration coefficients from PROM.
 */
void init_ms5611()
{
    /* Delay to allow proper sensor restart */
    delay_ms(5);

    /* Send reset command via SPI */
	cs_enable();
	SPI_Transmit((uint8_t[]){RESET},1);
	cs_disable();

    /* Wait 3 ms for reset to complete */
	delay_ms(3);

    /* Read calibration coefficients C1–C5 from PROM */
	ms5611_read_calibration();
}

/**
 * @brief Reads the current pressure from the MS5611.
 *        Gets high-resolution raw pressure and temperature values, then computes compensated pressure.
 * @return Compensated pressure in sensor units
 */
int32_t read_preassure(void)
{
    /* Acquire raw pressure value (D1) with 4096-bit precision */
	uint32_t D1 = get_raw_data(CMD_CONVERT_D1_OSR_4096);

    /* Acquire raw temperature value (D2) with 4096-bit precision */
	uint32_t D2 = get_raw_data(CMD_CONVERT_D2_OSR_4096);

    /* Calculate compensated pressure using calibration coefficients */
	return ms5611_calc_pressure(D1,D2);
}

/**
 * @brief Calculates compensated pressure using raw ADC values and calibration coefficients.
 * @param D1 Raw pressure value
 * @param D2 Raw temperature value
 * @return Compensated pressure
 */
static int32_t ms5611_calc_pressure(uint32_t D1, uint32_t D2)
{
    /* Temperature difference from reference */
	int32_t dT   =  (int32_t)D2 - ((int32_t)C5 << 8);

    /* Offset at actual temperature */
	int64_t OFF  =  ((int64_t)C2 << 16U) + (((int64_t)C4 * (int64_t)dT) >> 7U);

    /* Sensitivity at actual temperature */
	int64_t SENS =  ((int64_t)C1 << 15U) + (((int64_t)C3 * (int64_t)dT) >> 8U);

    /* Calculate compensated pressure */
	int64_t temp =  (((int64_t)D1 * SENS) >> 21U) - OFF;
	int32_t P 	 = 	(int32_t)(temp >> 15U);

	return P;
}


/**
 * @brief Reads calibration coefficients (C1–C5) from the sensor's PROM.
 */
static void ms5611_read_calibration()
{
    /* Read each calibration coefficient from PROM */
	C1 =  get_prom_coeff(PROM_C1_SENS_ADDR);	// Pressure sensitivity
	C2 =  get_prom_coeff(PROM_C2_OFF_ADDR);		// Pressure offset
	C3 =  get_prom_coeff(PROM_C3_TCS_ADDR);		// Temp coeff of pressure sensitivity
	C4 =  get_prom_coeff(PROM_C4_TCO_ADDR);		// Temp coeff of pressure offset
	C5 =  get_prom_coeff(PROM_C5_TREF_ADDR);	// Reference temperature
}


/**
 * @brief Reads raw ADC data from the MS5611 for a given conversion command.
 * @param command Conversion command (pressure or temperature)
 * @return 24-bit raw ADC value
 */
static uint32_t get_raw_data(uint8_t command)
{
	uint8_t buf[3];

    /* Start conversion by sending command */
	cs_enable();
	SPI_Transmit((uint8_t[]){command},1);
	cs_disable();

    /* Wait for conversion to complete (~9 ms) */
	vTaskDelay(pdMS_TO_TICKS(9));

    /* Read ADC result */
	cs_enable();
	SPI_Transmit((uint8_t[]){ADC_READ},1); // Command to read ADC
	SPI_Receive(buf,3);					   // Receive 3 bytes
	cs_disable();

    /* Combine 3 bytes into a 24-bit integer */
	uint32_t D = (((uint32_t)buf[0] << 16U) | ((uint32_t)buf[1] << 8U) | (uint32_t)buf[2]);
	
	return (D &= 0xFFFFFF);
}

/**
 * @brief Reads a 16-bit calibration coefficient from the PROM.
 * @param maddr Address of the coefficient in PROM
 * @return 16-bit coefficient
 */
static uint16_t get_prom_coeff(uint8_t maddr)
{
	uint8_t buf[2];

    /* Send PROM read command for the given address */
	cs_enable();
	SPI_Transmit((uint8_t[]){maddr},1);
	SPI_Receive(buf,2);  // Receive 2 bytes (16-bit coefficient)
	cs_disable();

    /* Combine MSB and LSB into 16-bit coefficient */
	return (((uint16_t)buf[0] << 8U) | (uint16_t)buf[1]);
}

