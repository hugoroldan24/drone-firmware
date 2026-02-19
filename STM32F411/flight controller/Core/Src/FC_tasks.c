
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * @file FC_tasks.c															 *
 * @brief																	 *
 * This file contains all task definitions	                                 *
 * required for the flight control system. It creates FreeRTOS tasks         *
 * and defines the main loop of each task that		                         *
 * handles joystick input, IMU processing, PID computation, motor mixing,	 *
 * telemetry, sensor reading, and safety/failsafe mechanisms.				 *
 *																			 *
 * Tasks are created with specific priorities to ensure the real-time		 *
 * constraints of the flight controller, such as achieving the 5 ms control  *
 * loop for motor updates.													 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "dron_structs.h"
#include "const.h"
#include "drivers.h"
#include "motor_mixer.h"
#include "pid_control.h"
#include "filter.h"
#include "telemetry.h"
#include "failsafe.h"
#include "joystick_mapper.h"
#include "battery_lecture.h"
#include "telemetry.h"
#include "util.h"
#include <math.h>

static drone_state_t system_state = STATE_BOOT;


       void xHandleRCRxTask	     (void* parameters);
static void xHandleIMUTask	     (void* parameters);
static void xHandleControlTask	 (void* parameters);
static void xHandleSensorsTask	 (void* parameters);
static void xHandleTelemetryTask (void* parameters);
static void xHandleSafetyTask	 (void* parameters);
static void xHandlePIDErrorTask	 (void* parameters);

static void set_pid_params(PID_Controller_t *roll, PID_Controller_t *pitch, PID_Controller_t *yaw);
static void update_PWM_Outputs(TIM_TypeDef* timer, PWM_Outputs_t ccr);
static Status_t checkIMU(float imu_data[6]);


TaskHandle_t      RC_RX_ID, IMU_ID, CTRL_ID, SENSORS_ID, TELEM_ID, SAFETY_ID, PID_ERROR_ID,SYSTEM_STATE_ID;
PWM_Outputs_t     ccr;
QueueHandle_t 	  telem_queue_ID, flight_data_queue_ID, pid_error_queue_ID, system_event_queue_ID;
SemaphoreHandle_t MPU_Semaphore, IMU_RX_SYN_Semaphore;
UserControl_t 	  joystick_data;
static uint8_t    FAILSAFE_ACTIVATED;


/* Buffer that the task being created will use as its stack. Note this is
       an array of StackType_t variables. The size of StackType_t is dependent on
       the RTOS port. */
StackType_t   IMU_Task_Stack          [STACK_SIZE_WORDS];
StackType_t   CTRL_Task_Stack         [STACK_SIZE_WORDS];
StackType_t   SENSORS_Task_Stack      [STACK_SIZE_WORDS];
StackType_t   TELEM_Task_Stack        [STACK_SIZE_WORDS];
StackType_t   SAFETY_Task_Stack       [STACK_SIZE_WORDS];
StackType_t   PID_ERRORS_Task_Stack   [STACK_SIZE_WORDS];
StackType_t   SYSTEM_STATE_Task_Stack [STACK_SIZE_WORDS];


/* Declare static buffers thath will be used to hold the task's data structures (TCB), has to be persistent*/
StaticTask_t IMU_TCB;
StaticTask_t CTRL_TCB;
StaticTask_t SENSORS_TCB;
StaticTask_t TELEM_TCB;
StaticTask_t SAFETY_TCB;
StaticTask_t PID_ERRORS_TCB;
StaticTask_t SYSTEM_STATE_TCB;

/*Arrays that are at least large enough to hold the maximum number of items that can be in the queues at any one time*/ 
uint8_t telem_queue        [TELEM_QUEUE_NUM_ELEM        * sizeof(Telemetry_t)];
uint8_t flight_data_queue  [FLIGHT_DATA_QUEUE_NUM_ELEM  * sizeof(FlightMessage_t)];
uint8_t pid_error_queue    [PID_ERROR_QUEUE_NUM_ELEM    * sizeof(FlightMessage_t)];
uint8_t system_event_queue [SYSTEM_EVENT_QUEUE_NUM_ELEM * sizeof(SystemEvent_t)];

/* Used to hold the queue's data structure statically.*/
StaticQueue_t telem_queue_struct;
StaticQueue_t flight_data_queue_struct;
StaticQueue_t pid_error_queue_struct;
StaticQueue_t system_event_queue_struct;

/* Will be used to hold the semaphore's state.*/
StaticSemaphore_t MPU_Semaphore_State;
StaticSemaphore_t IMU_RX_Sync_Semaphore_State;


/**
 * @brief Create all FreeRTOS tasks and related synchronization primitives.
 *
 * This function creates tasks for RC input, IMU processing, PID control,
 * sensor reading, telemetry transmission, safety handling, and PID error
 * computation. It also creates queues and semaphores for inter-task communication.
 */
void createTasks()
{
    /* Create RC receiver task (medium priority) */
	/* This task is created dinamically because it can be deleted and created dinamically */
    xTaskCreate(xHandleRCRxTask,"RC_RX",200,NULL,2,&RC_RX_ID);

    /* Create IMU processing task (medium priority) */
	IMU_ID = xTaskCreate(xHandleIMUTask,"IMU",
		   STACK_SIZE_WORDS,NULL,2,IMU_Task_Stack,&IMU_TCB);

    /* Create Control task for PID computation and motor output (high priority) */
	CTRL_ID = xTaskCreateStatic(xHandleControlTask,"CTRL",
		   STACK_SIZE_WORDS,NULL,5,CTRL_Task_Stack,&CTRL_TCB);

    /* Create Sensors task to read battery and pressure sensors (low priority) */
	SENSORS_ID = xTaskCreateStatic(xHandleSensorsTask,"SENSORS",
		   STACK_SIZE_WORDS,NULL,1,SENSORS_Task_Stack,&SENSORS_TCB);

    /* Create Telemetry task to send data via UART (lowest priority) */
	TELEM_ID = xTaskCreateStatic(xHandleTelemetryTask,"TELEM",
		   STACK_SIZE_WORDS,NULL,0,TELEM_Task_Stack,&TELEM_TCB);

    /* Create Safety/Failsafe task (high priority) */
	SAFETY_ID = xTaskCreateStatic(xHandleSafetyTask,"SAFETY",
		   STACK_SIZE_WORDS,NULL,4,SAFETY_Task_Stack,&SAFETY_TCB);

    /* Create PID error computation task (medium-high priority) */
	PID_ERROR_ID = xTaskCreateStatic(xHandlePIDErrorTask,"ERROR",
		   STACK_SIZE_WORDS,NULL,3,PID_ERRORS_Task_Stack,&PID_ERRORS_TCB);

	/* Create system state task (higher priority) */
	SYSTEM_STATE_ID = xTaskCreateStatic(xHandleSystemStateTask,"STATE",
	       STACK_SIZE_WORDS,NULL,6,SYSTEM_STATE_Task_Stack,&SYSTEM_STATE_TCB);

    /* Create queues for inter-task communication */
	telem_queue_ID        = xQueueCreateStatic(TELEM_QUEUE_NUM_ELEM,sizeof(Telemetry_t),
	                       telem_queue,&telem_queue_struct); 

	flight_data_queue_ID  = xQueueCreateStatic(FLIGHT_DATA_QUEUE_NUM_ELEM,sizeof(FlightMessage_t),
                           flight_data_queue,&flight_data_queue_struct);

	pid_error_queue_ID    = xQueueCreateStatic(PID_ERROR_QUEUE_NUM_ELEM,sizeof(FlightMessage_t),
                           pid_error_queue,&pid_error_queue_struct);
	
	system_event_queue_ID = xQueueCreateStatic(SYSTEM_EVENT_QUEUE_NUM_ELEM,sizeof(SystemEvent_t),
                           system_event_queue,&system_event_queue_struct);

    /* Semaphore for IMU access */
	MPU_Semaphore        = xSemaphoreCreateBinaryStatic(&MPU_Semaphore_State);

	IMU_RX_SYN_Semaphore = xSemaphoreCreateBinaryStatic(&IMU_RX_Sync_Semaphore_State);
}


static void xHandleSystemStateTask(void* parameters)
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
			   vTaskSuspend(SENSORS_ID);  
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
                vTaskResume(SENSORS_ID);
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

   while(1)
   {
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
	           xQueueSend(system_event_queue,(void *)&event,portMAX_DELAY);
			}
		 break;

		 case STATE_FLIGHT:
		 break;

		 case STATE_LANDING:
		 break;
      }
   }
	/* Suspend self until activated */
	vTaskSuspend(NULL);

	

	if(!FAILSAFE_ACTIVATED){
		while(1)
		{
            xSemaphoreTake(IMU_RX_SYN_Semaphore);

            /* Read joystick axes */
			get_joystick_data(&joystick_data);

            /* Convert joystick to flight setpoints */
			map_joystick_to_setpoint(joystick_data, &setpoint);

            /* Send setpoint to flight data queue */
			xQueueSend(flight_data_queue,(void *)&setpoint,portMAX_DELAY);     
		}
	}
	else{
        /* Manual override: simulate joystick at rest */
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
	}
}


/**
 * @brief Task handling IMU measurements.
 *
 * Reads accelerometer and gyroscope data from the MPU6050, computes
 * attitude, sends it to the flight data queue, and notifies the RC task.
 */
static void xHandleIMUTask(void* parameters)
{
    float imu_data[6];
	uint32_t lastcall;
	Status_t status = STATUS_OK;
	FlightMessage_t imu_attitude;
	SystemEvent_t event;
	

    /* Throttle not used for IMU data */
	imu_attitude.throttle = 0;

	while(1)
	{
		switch(system_state)
		{
			case STATE_BOOT:
			    /* Suspend self until activated */
	           vTaskSuspend(NULL);

			   /* Initialize MPU sensor */
	           init_mpu6050();
               event = EVENT_IMU_INITIALIZED;
	           xQueueSend(system_event_queue,(void *)&event,portMAX_DELAY);
			break;

			case STATE_STANDBY:
			   /* Read accelerometer and gyroscope */
	           status = mpu_read_acc_gyr(imu_data); 
			   status = checkIMU(imu_data);

			   if(status == STATUS_OK)
			   {
			      event = EVENT_IMU_VERIFIED;
	              xQueueSend(system_event_queue,(void *)&event,portMAX_DELAY);
			   }

			break;

			case STATE_FLIGHT:
			break;

			case STATE_LANDING:
			break;
		}
	}


	/* Wake up the higher priority task now that we have initialized all the peripherals
	that required the scheduler to be set.*/
	vTaskResume(CTRL_ID); 
	taskYIELD();

	lastcall = get_time_now_us();

	while(1)
	{
       /* Read accelerometer and gyroscope */
	   mpu_read_acc_gyr(imu_data);

       /* Compute pitch, roll, yaw */
	   computeAttitudeFromIMU(imu_data, &imu_attitude.attitude,&lastcall);

	   /* Notify RC task that IMU data is ready */
	   xSemaphoreGive(IMU_RX_SYN_Semaphore);

       /* Send attitude data */
	   xQueueSend(flight_data_queue,(void *)&imu_attitude,portMAX_DELAY);

       /* Wait until CTRL task sends notification to start again */
	   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
	}
}

/**
 * @brief Task computing PID error for roll, pitch, and yaw_rate.
 *
 * Receives setpoints and IMU data from the flight data queue, computes
 * the PID error, and sends it to the PID error queue for the control task.
 */
static void xHandlePIDErrorTask(void* parameters)
{
	while(1)
	{
		switch(system_state)
		{
			case STATE_BOOT:
			    /* Suspend self until activated */
	           vTaskSuspend(NULL);
			break;

			case STATE_STANDBY:
			/* The task should not been resumed in this state */
			break;

			case STATE_FLIGHT:
			break;

			case STATE_LANDING:
			break;
		}
	}
	/* Suspend self until activated */
	vTaskSuspend(NULL);

	FlightMessage_t rx_setpoint, rx_imu, pid_error;

	while(1){
        /* Receive IMU data from queue */
		xQueueReceive(flight_data_queue, &rx_imu, portMAX_DELAY);

        /* Receive setpoint from queue */
		xQueueReceive(flight_data_queue, &rx_setpoint, portMAX_DELAY);

        /* Preserve throttle for control task */
		pid_error.throttle = rx_setpoint.throttle;

        /* Compute PID error */
		computePIDError(rx_setpoint.attitude, rx_imu.attitude, &pid_error.attitude);

        /* Send PID error to control task */
		xQueueSend(pid_error_queue, ( void * )&pid_error,portMAX_DELAY);
	}
}

/**
 * @brief Task for PID control and motor mixing.
 *
 * Receives PID errors, computes PID outputs for roll, pitch, and yaw_rate,
 * mixes outputs into motor commands, and updates PWM signals through
 * the motor mixer.
 */
static void xHandleControlTask(void* parameters)
{
	while(1)
	{
		switch(system_state)
		{
			case STATE_BOOT:
	 		   /* Suspend self until activated */
	           vTaskSuspend(NULL);
			break;

			case STATE_STANDBY: 
			/* The task should not been resumed in this state */
			break;

			case STATE_FLIGHT:
			break;

			case STATE_LANDING:
			break;
		}
	}
	/* Suspend self until activated */
	vTaskSuspend(NULL);

	/* Resume the remaining tasks to start normal execution */
	vTaskResume(PID_ERROR_ID);
	vTaskResume(RC_RX_ID);

	FlightMessage_t  pid_error;
	PID_Controller_t roll_pid_params ;
	PID_Controller_t pitch_pid_params;
	PID_Controller_t yaw_pid_params;
	PID_Outputs_t    output;

	uint32_t lastcall = 0;
	float dt;

	const TickType_t xFrequency = pdMS_TO_TICKS(3U);

	set_pid_params(&roll_pid_params, &pitch_pid_params, &yaw_pid_params);
    xLastWakeTime = xTaskGetTickCount();

	start_PWM();
	lastcall = get_time_now_us();
	while(1)
	{
		xQueueReceive(pid_error_queue,&pid_error,portMAX_DELAY);

		dt = get_time_elapsed(&lastcall);

		output.roll 	= compute_PID(pid_error.attitude.roll,     dt, &roll_pid_params);
		output.pitch 	= compute_PID(pid_error.attitude.pitch,    dt, &pitch_pid_params);
		output.yaw_rate = compute_PID(pid_error.attitude.yaw_rate, dt, &yaw_pid_params);

		motor_mixer(output,pid_error.throttle,&ccr); 
		
		/* Make sure that all the other tasks except telemetry ones are blocked
		so that in this delay time they can execute (or IDLE to save power)*/
		vTaskDelayUntil(&xLastWakeTime,xFrequency); 

		update_PWM_Outputs(TIM3,ccr);

		/* Signal IMU task to start a new cycle */
		xTaskNotifyGive(IMU_ID);
	}
}

/**
 * @brief Task handling sensor readings.
 *
 * Reads battery voltage and pressure sensors at a fixed interval (200 ms),
 * and sends the data to the telemetry queue.
 */
static void xHandleSensorsTask(void* parameters)
{
	Telemetry_t data;
	TickType_t xLastWakeTime;
    SystemEvent_t event;

	const TickType_t xFrequency = pdMS_TO_TICKS(200U);
 
	while(1)
	{
		switch(system_state)
		{
			case STATE_BOOT:
			   init_ms5611();
	           event = EVENT_BAR_INITIALIZED;
	           xQueueSend(system_event_queue,(void *)&event,portMAX_DELAY);
			break;

			case STATE_STANDBY:
			   
			break;

			case STATE_FLIGHT:
			break;

			case STATE_LANDING:
			break;
		}
	}
	

    xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		data.preassure     = read_preassure();
		data.battery_level = read_battery();
		xQueueSend(telem_queue, (void *)&data,portMAX_DELAY); 
		vTaskDelayUntil(&xLastWakeTime,xFrequency); 
	}
}

/**
 * @brief Task handling telemetry transmission.
 *
 * Receives telemetry data from the sensor queue and sends it via UART DMA.
 */
static void xHandleTelemetryTask(void* parameters)
{
	Telem_t rx_data;
	while(1)
	{
		switch(system_state)
		{
			case STATE_BOOT:
			   /* Suspend self until activated */
	           vTaskSuspend(NULL); 
			break;

			case STATE_STANDBY:
			break;

			case STATE_FLIGHT:
			break;

			case STATE_LANDING:
			break;
		}
	}
    /* This task is not suspended because it has the lowest priority so does not matter
	if we suspend it*/
	
	while(1)
	{
		xQueueReceive(telem_queue, &rx_data,portMAX_DELAY); 
		send_telemetry(rx_data);
	}
}


/**
 * @brief Safety/failsafe task.
 *
 * Executes failsafe procedures when triggered, such as stopping
 * manual control, disabling peripherals, and performing hover or landing.
 */
static void xHandleSafetyTask(void* parameters)
{
	while(1)
	{
		switch(system_state)
		{
			case STATE_BOOT:
		       /* Suspend self until activated */
	           vTaskSuspend(NULL);
			break;

			case STATE_STANDBY:
			/* The task should not been resumed in this state */
			break;

			case STATE_FLIGHT:
			break;

			case STATE_LANDING:
			break;
		}
	}
    /* Suspend self until activated */
	vTaskSuspend(NULL);

    /* Indicate failsafe mode active */
	FAILSAFE_ACTIVATED = 1;

    /* Suspend tasks that we do not need anymore */
	vTaskSuspend(SENSORS_ID);

	vTaskSuspend(TELEM_ID);

    /* Disable unnecessary peripherals */
	disable_periph();

    /* Enter hover mode without user control */
	set_hover();

    /* Execute autonomous landing procedure */
	failsafe_execute_landing();
}



/**
 * @brief Idle task hook called by FreeRTOS when no other task is running.
 *        Puts the MCU into sleep mode to reduce power consumption. To execute 
 *        this handler, configUSE_IDLE_HOOK flag must be set to 1.
 */
void vApplicationIdleHook(void)
{
	 /* Put the MCU into low-power sleep mode until the next interrupt
	       SLEEPDEEP bit is 0, so MCU enters standard sleep (not deep sleep) */
	__WFI();
}


/**
 * @brief Static function that initialices the pid parameters structs to the desired values
 * defined in "const.h"        
 */
static void set_pid_params(PID_Controller_t *roll, PID_Controller_t *pitch, PID_Controller_t *yaw)
{
   roll->kp = ROLL_KP;
   roll->ki = ROLL_KI;
   roll->kd = ROLL_KD;
   roll->output_min = ROLL_OUTPUT_MIN;
   roll->output_max = ROLL_OUTPUT_MAX;

   pitch->kp = PITCH_KP;
   pitch->ki = PITCH_KI;
   pitch->kd = PITCH_KD;
   pitch->output_min = PITCH_OUTPUT_MIN;
   pitch->output_max = PITCH_OUTPUT_MAX;

   yaw->kp = YAW_KP;
   yaw->ki = YAW_KI;
   yaw->kd = YAW_KD;
   yaw->output_min = YAW_OUTPUT_MIN;
   yaw->output_max = YAW_OUTPUT_MAX;
}


/**
 * @brief Updates the PWM output registers of a given timer.
 * 
 * This function writes the PWM duty cycle values to the CCR (Capture/Compare) registers
 * of the specified timer. Each CCR channel corresponds to one motor's ESC on a quadcopter.
 * 
 * @param timer Pointer to the TIM peripheral (e.g., TIM1, TIM3) whose CCR registers
 *              will be updated.
 * @param ccr   Structure containing the PWM duty cycle values for each motor:
 *              - motor1_pwm → CCR1
 *              - motor2_pwm → CCR2
 *              - motor3_pwm → CCR3
 *              - motor4_pwm → CCR4
 * 
 * @note The timer must be properly configured for PWM mode before calling this function.
 *       The values in the CCR registers determine the duty cycle and thus the speed
 *       of each motor via its ESC.
 */
static void update_PWM_Outputs(TIM_TypeDef* timer, PWM_Outputs_t ccr)
{
   timer->CCR1 = ccr.motor1_pwm;
   timer->CCR2 = ccr.motor2_pwm;
   timer->CCR3 = ccr.motor3_pwm;
   timer->CCR4 = ccr.motor4_pwm;

   /* Start timer to generate PWM signals (OneShot125) */
   timer->CR1 |= TIM_CR1_CEN;
}

static Status_t checkIMU(float imu_data[6U])
{
   uint8_t i;
   for(i = 0U; i < 3U ;i++)
   {
       if(fabs(imu_data[i]) > ACC_TOLERANCE || fabs(imu_data[i+3]) > GYRO_TOLERANCE)
	   {
	      return STATUS_NOT_OK; //QUiere decir que dron no está quieto
	   }	   
   }
   return STATUS_OK;
}



