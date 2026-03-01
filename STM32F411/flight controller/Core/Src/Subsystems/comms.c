#include "FreeRTOS.h"
#include "config.h"
#include "state_manager.h"


static uint8_t get_joystick_data(UserControl_t* input);
static void map_joystick_to_setpoint(UserControl_t input, FlightMessage_t* setpoint);

extern QueueHandle_t system_event_queue_ID, flight_data_queue_ID;
extern SemaphoreHandle_t IMU_RX_SYN_Semaphore;


/**
 * @brief Task handling RC receiver input.
 *
 * Waits for joystick notifications, reads joystick data, maps it to flight
 * setpoints, and sends it to the flight data queue. Notifies the IMU task.
 */
void xHandleRCRxTask(void* parameters)
{
   FlightMessage_t setpoint;
   SystemEvent_t event;
   Status_t status = STATUS_OK;
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
		    /* Read joystick axes just to verify proper connection */
			status = get_joystick_data(&joystick_data);
            if(status == STATUS_OK)
			{
               event = EVENT_RX_VERIFIED;
	           xQueueSend(system_event_queue_ID,(void *)&event,portMAX_DELAY);
			}
		 break;

		 case STATE_FLIGHT:
           xSemaphoreTake(IMU_RX_SYN_Semaphore);

           /* Read joystick axes */
		   get_joystick_data(&joystick_data);

           /* Convert joystick to flight setpoints */
		   map_joystick_to_setpoint(joystick_data, &setpoint);

           /* Send setpoint to flight data queue */
		   xQueueSend(flight_data_queue,(void *)&setpoint,portMAX_DELAY);     
		    
		 break;

		 case STATE_LANDING:
		    joystick_data = (UserControl_t){
		    .left_x_axis  = JOY_MID,
		    .left_y_axis  = HOVER_THROTTLE,
		    .right_x_axis = JOY_MID,
		    .right_y_axis = JOY_MID
		   };
		   while(1)
		   {
		      xSemaphoreTake(IMU_RX_SYN_Semaphore);
			  map_joystick_to_setpoint(joystick_data, &setpoint);
			  xQueueSend(flight_data_queue,(void *)&setpoint,portMAX_DELAY);
		   }
		 break;
      }
   }
}


/**
 * @brief Reads joystick data from UART into the UserControl structure.
 * @param input Pointer to UserControl_t structure to store joystick axis values
 */
static uint8_t get_joystick_data(UserControl_t* input)
{
	/* Read joystick bytes from UART via DMA into the axis array.
	       FRAME_LEN-1 excludes protocol-specific start byte and checksum.
	       Order of bytes is important to match channel mapping. */
	return uart1_read_dma(input->axis,FRAME_LEN-1);
}


/**
 * @brief Maps the raw joystick axes to drone control setpoints.
 *        Left joystick: Y → throttle, X → yaw rate
 *        Right joystick: Y → pitch, X → roll
 * @param input UserControl_t structure containing raw joystick axes
 * @param setpoint Pointer to FlightMessage_t structure to store mapped setpoints
 */
static void map_joystick_to_setpoint(UserControl_t input, FlightMessage_t* setpoint)
{
	setpoint->attitude.roll     =    MAP_JOYSTICK_TO_ANGLE(input.right_x_axis);   /* Map right joystick horizontal (X) to roll angle */
	setpoint->attitude.pitch    = 	 MAP_JOYSTICK_TO_ANGLE(input.right_y_axis);   /* Map right joystick vertical (Y) to pitch angle */
	setpoint->attitude.yaw_rate = 	 MAP_JOYSTICK_TO_YAW(input.left_x_axis);      /* Map left joystick horizontal (X) to yaw rate */
	setpoint->throttle 			=    MAP_JOYSTICK_TO_THROTTLE(input.left_y_axis); /* Map left joystick vertical (Y) to throttle */
}