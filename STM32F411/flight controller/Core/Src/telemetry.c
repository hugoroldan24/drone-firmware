/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * @file telemetry.c																			   *
 * @brief																						   *
 * This module receives raw telemetry data (battery ADC value and raw pressure),				   *
 * converts it into meaningful values (battery voltage per cell and relative altitude),		       *
 * and sends the processed data via UART using DMA.												   *
 *																								   *
 * Battery voltage is checked against a safety threshold; if too low, the safety task is resumed.  *
 * Relative altitude is computed using the barometric formula, with the first pressure			   *
 * reading taken as reference.																	   *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "battery_lecture.h"
#include "MS5611.h"
#include "uart.h"
#include "math.h"
#include "const.h"
#include "dron_structs.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

static void ProcessBatteryAndAltitude(uint16_t raw_battery, int32_t raw_preassure, uint8_t telemetry_data[]);

extern TaskHandle_t SAFETY_ID;

/**
 * @brief Sends the telemetry data over UART.
 *
 * This function converts the raw battery and pressure readings into voltage per cell
 * and relative altitude, packs the results into a 4-byte frame, and transmits it
 * via UART using DMA. It calls ProcessBatteryAndAltitude() internally.
 *
 * @param data The raw telemetry data structure containing battery level and pressure.
 */
void send_telemetry(Telem_t data)
{
	static uint8_t telemetry_data[4];

	/* Convert raw battery and pressure values to meaningful voltage and altitude */
	ProcessBatteryAndAltitude(data.battery_level,data.preassure,telemetry_data);

	/* Send telemetry data via UART using DMA */
	uart1_send_dma(telemetry_data,TELEM_FRAME_SIZE);
}

/**
 * @brief Processes raw battery and pressure readings into usable telemetry values.
 *
 * Converts the raw ADC battery value to voltage per cell and the raw pressure
 * reading to relative altitude in meters. Updates the telemetry_data array with
 * the integer and fractional parts of both measurements. If battery voltage is below
 * a defined minimum, the failsafe task is resumed.
 *
 * @param raw_battery Raw ADC value of the battery.
 * @param raw_preassure Raw pressure reading.
 * @param telemetry_data Output array containing processed telemetry values.
 */
static void ProcessBatteryAndAltitude(uint16_t raw_battery, int32_t raw_preassure, uint8_t telemetry_data[])
{
	static uint8_t first; /* Flag to check if initial pressure reference has been set */
	static int32_t Pref;  /* Initial pressure reference used to calculate relative height */

	/* Convert raw ADC battery reading into voltage per cell */
	float battery_voltage = (((float)raw_battery / ADC_MAX_VALUE)*VREF*DIVIDER_FACTOR)/3; /* Divide by 3 to get the voltage per cell */

	/* Trigger safety task if battery voltage drops below the established minimum */
	if(battery_voltage <= BATTERY_MIN_V)
	{
		vTaskResume(SAFETY_ID);
	}

	/* Set initial pressure reference on first execution */
	if(!first){
		Pref = raw_preassure;
		first = 1;
	}
	/* Convert pressure to relative height in meters using standard formula */
	float height = 44330.0f * (1.0f - powf(raw_preassure / Pref,0.1903f));

	/* Store voltage and height as integer + first decimal in telemetry_data array */
	telemetry_data[0]  =  (uint8_t)battery_voltage;
	telemetry_data[1]  =  (uint8_t)((battery_voltage-telemetry_data[0])*10);
	telemetry_data[2]  =  (uint8_t)height;
	telemetry_data[3]  =  (uint8_t)((height-telemetry_data[2])*10);
}



