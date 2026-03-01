#include "dron_structs.h"
#include "config.h"

extern TaskHandle_t      RC_RX_ID, IMU_ID, CTRL_ID, MONITOR_ID, TELEM_ID, SAFETY_ID, PID_ERROR_ID;
static DroneState_t     system_state = STATE_BOOT;

DroneState_t get_drone_state(void) {
    return system_state;
}


void xHandleSystemStateTask(void* parameters)
{
   SystemEvent_t new_event;
   SystemEvent_t curr_event = EVENT_POWERED_ON;
   while(1)
   {
	  
	  xQueueReceive(system_event_queue, &new_event,portMAX_DELAY); 
	  curr_event = new_event;
	  
	  if(curr_event != new_event)
	  {
		 curr_event = new_event;

         switch(system_state)
         {
            case STATE_BOOT:			   
	           if(curr_event == EVENT_BAR_INITIALIZED)
			   {
			      vTaskResume(IMU_ID);
			      vTaskSuspend(MONITOR_ID);  
			   }
			   else if(curr_event == EVENT_IMU_INITIALIZED)
			   {
                  system_state = STATE_STANDBY;
			      vTaskSuspend(IMU_ID);
			      vTaskResume(RC_RX_ID);
			   }
            break;
	        case STATE_STANDBY:
		       if(curr_event == EVENT_RX_VERIFIED)
			   {
			      /* Suspend */
	              vTaskSuspend(RC_RX_ID);	
			      vTaskResume(IMU_ID);
			   }
			   if(curr_event == EVENT_IMU_VERIFIED)
			   {
			      vTaskSuspend(IMU_ID);
                  vTaskResume(MONITOR_ID);
			   }
			   if(curr_event == EVENT_BATTERY_VERIFIED)
			   {
                  vTaskSuspend(MONITOR_ID);			   
			   }
			   if(curr_event == EVENT_TAKEOFF)
			   {
			      system_state = STATE_FLIGHT;
				  vTaskResume(IMU_ID);
				  vTaskResume(RC_RX_ID);
				  vTaskResume(MONITOR_ID);
				  vTaskResume(CTRL_ID);
				  vTaskResume(TELEM_ID);
				  vTaskResume(PID_ERROR_ID);
			   }
	        break;
	        case STATE_FLIGHT:
	        break;
	        case STATE_LANDING:
	        break;
         }
      }
   }
}