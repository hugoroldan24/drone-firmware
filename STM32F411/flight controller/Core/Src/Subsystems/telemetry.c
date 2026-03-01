#include "stm32f411xe.h"
#include "config.h"
#include "stdint.h"
#include "util.h"
#include "dron_structs.h"


extern QueueHandle_t telem_queue_ID;
/**
 * @brief Task handling telemetry transmission.
 *
 * Receives telemetry data from the sensor queue and sends it via UART DMA.
 */
static void xHandleTelemetryTask(void* parameters)
{
	Raw_Data_t rx_data;
	Telem_t telem_data;
    DroneState_t system_state;

	while(1)
	{
        system_state = get_drone_state();
		switch(system_state)
		{
			case STATE_BOOT:
			   /* Suspend self until activated */
	           vTaskSuspend(NULL); 
			break;

			case STATE_STANDBY:
			   /* Do nothing, should not reach this state */
			break;

			case STATE_FLIGHT:
			   xQueueReceive(telem_queue_ID, &rx_data,portMAX_DELAY); 

		       /* Convert raw battery and pressure values to meaningful voltage and altitude */
	           ProcessBatteryAndAltitude(rx_data,&telem_data);

	           send_telemetry(&telem_data);
			break;

			case STATE_LANDING:
			break;
		}
	}
}

/**
 * @brief Sends the telemetry data over UART.
 *
 * This function converts the raw battery and pressure readings into voltage per cell
 * and relative altitude, packs the results into a 4-byte frame, and transmits it
 * via UART using DMA. It calls ProcessBatteryAndAltitude() internally.
 *
 * @param data The raw telemetry data structure containing battery level and pressure.
 */
void send_telemetry(Telem_t *telem_data)
{
	/* Send telemetry data via UART using DMA */
	uart1_send_dma(telem_data->telem_array,TELEM_FRAME_SIZE);
}

/**
 * @brief Processes raw battery and pressure readings into usable telemetry values.
 *
 * Converts the raw ADC battery value to voltage and the raw pressure
 * reading to relative altitude in meters. Updates the telemetry_data array with
 * the integer and fractional parts of both measurements. If battery voltage is below
 * a defined minimum, the failsafe task is resumed.
 *
 * @param raw_battery Raw ADC value of the battery.
 * @param raw_preassure Raw pressure reading.
 * @param telemetry_data Output array containing processed telemetry values.
 */
void ProcessBatteryAndAltitude(Raw_Data_t rx_data, Telem_t *telem_data)
{
	static uint8_t first; /* Flag to check if initial pressure reference has been set */
	static int32_t Pref;  /* Initial pressure reference used to calculate relative height */

	/* Convert raw ADC battery reading into voltage */
	float battery_voltage = (((float)rx_data.raw_battery * VREF) / ADC_MAX_VALUE) * DIVIDER_FACTOR;  /* Get the battery voltage */

	/* Trigger safety task if battery voltage drops below the established minimum */
	if(battery_voltage <= BATTERY_MIN_V)
	{
		vTaskResume(SAFETY_ID);
	}

	/* Set initial pressure reference on first execution */
	if(!first){
		Pref = rx_data.preassure;
		first = 1;
	}
	/* Convert pressure to relative height in meters using standard formula */
	float altitude = 44330.0f * (1.0f - powf(rx_data.raw_preassure / Pref,0.1903f));

	/* Store voltage and height as integer + first decimal in telemetry_data array */
	telem_data->battery_int   =  (uint8_t)battery_voltage;
	telem_data->battery_frac  =  (uint8_t)((battery_voltage - (uint8_t)battery_voltage) * 10U);
	telem_data->altitude_int  =  (uint8_t)altitude;
	telem_data->altitude_frac =  (uint8_t)((height - (uint8_t) altitude) * 10U);
}