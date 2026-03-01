#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "dron_structs.h"
#include "config.h"
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



extern void xHandleRCRxTask	     (void* parameters);
extern void xHandleIMUTask	     (void* parameters);
extern void xHandleControlTask	 (void* parameters);
extern void xHandleSensorsTask	 (void* parameters);
extern void xHandleTelemetryTask (void* parameters);
extern void xHandleSafetyTask	 (void* parameters);
extern void xHandlePIDErrorTask	 (void* parameters);


static Status_t checkIMU(float imu_data[6]);
static Status_t checkBattery(uint16_t raw_battery);


TaskHandle_t      RC_RX_ID, IMU_ID, CTRL_ID, MONITOR_ID, TELEM_ID, SAFETY_ID, PID_ERROR_ID,SYSTEM_STATE_ID;
PWM_Outputs_t     ccr;
QueueHandle_t 	  telem_queue_ID, flight_data_queue_ID, pid_error_queue_ID, system_event_queue_ID;
SemaphoreHandle_t MPU_Semaphore, IMU_RX_SYN_Semaphore;
UserControl_t 	  joystick_data;
static uint8_t    FAILSAFE_ACTIVATED;


/* Buffer that the task being created will use as its stack. Note this is
       an array of StackType_t variables. The size of StackType_t is dependent on
       the RTOS port. */
StackType_t   RC_RX_Task_Stack       [STACK_SIZE_WORDS];
StackType_t   IMU_Task_Stack          [STACK_SIZE_WORDS];
StackType_t   CTRL_Task_Stack         [STACK_SIZE_WORDS];
StackType_t   SENSORS_Task_Stack      [STACK_SIZE_WORDS];
StackType_t   TELEM_Task_Stack        [STACK_SIZE_WORDS];
StackType_t   SAFETY_Task_Stack       [STACK_SIZE_WORDS];
StackType_t   PID_ERRORS_Task_Stack   [STACK_SIZE_WORDS];
StackType_t   SYSTEM_STATE_Task_Stack [STACK_SIZE_WORDS];


/* Declare static buffers that will be used to hold the task's data structures (TCB), has to be persistent*/
StaticTask_t RC_RX_TCB;
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
    RC_RX_ID = xTaskCreateStatic(xHandleRCRxTask,"RC_RX",
		   STACK_SIZE_WORDS,NULL,2,RC_RX_Task_Stack,&RC_RX_TCB);

    /* Create IMU processing task (medium priority) */
	IMU_ID = xTaskCreateStatic(xHandleIMUTask,"IMU",
		   STACK_SIZE_WORDS,NULL,2,IMU_Task_Stack,&IMU_TCB);

    /* Create Control task for PID computation and motor output (high priority) */
	CTRL_ID = xTaskCreateStatic(xHandleControlTask,"CTRL",
		   STACK_SIZE_WORDS,NULL,5,CTRL_Task_Stack,&CTRL_TCB);

    /* Create Sensors task to read battery and pressure sensors (low priority) */
	MONITOR_ID = xTaskCreateStatic(xHandleSensorsTask,"SENSORS",
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