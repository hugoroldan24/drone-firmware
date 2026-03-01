#include "stm32f411xe.h"
#include "config.h"
#include "stdint.h"
#include "util.h"
#include "dron_structs.h"


extern QueueHandle_t  system_event_queue_ID, telem_queue_ID;


/**
 * @brief Task handling sensor readings.
 *
 * Reads battery voltage and pressure sensors at a fixed interval (200 ms),
 * and sends the data to the telemetry queue.
 */
static void xHandleMonitorTask(void* parameters)
{
	Raw_Data_t raw_data;
	TickType_t xLastWakeTime;
    SystemEvent_t event;
	Status_t status;

	const TickType_t xFrequency = pdMS_TO_TICKS(200U);
 
	while(1)
	{
		switch(system_state)
		{
			case STATE_BOOT:
			   init_ms5611();
	           event = EVENT_BAR_INITIALIZED;
	           xQueueSend(system_event_queue_ID,(void *)&event,portMAX_DELAY);
			break;

			case STATE_STANDBY:
			    
               raw_data.battery_level = read_battery();
               
			   status = checkBattery(data.battery_level);

			   if(status == STATUS_OK)
			   {				  
				  event = EVENT_BATTERY_VERIFIED;
			      xQueueSend(system_event_queue_ID,(void *)&event,portMAX_DELAY);
			   }
			   
			break;

			case STATE_FLIGHT:
			   xLastWakeTime = xTaskGetTickCount();
			   while(1)
			   {
                  raw_data.preassure     = read_preassure();
		          raw_data.battery_level = read_battery();

		          xQueueSend(telem_queue_ID, (void *)&raw_data,portMAX_DELAY); 
		          vTaskDelayUntil(&xLastWakeTime,xFrequency); 				
			   } 
			break;

			case STATE_LANDING:
			break;
		}
	}
}


static Status_t checkBattery(uint16_t raw_battery)
{   
   float battery_voltage;

   battery_voltage = (((float)raw_battery * VREF) / ADC_MAX_VALUE) * DIVIDER_FACTOR;

   if(battery_voltage >= BATTERY_NOMINAL_V)
   {
      return STATUS_OK;
   }
   return STATUS_NOT_OK;
}
