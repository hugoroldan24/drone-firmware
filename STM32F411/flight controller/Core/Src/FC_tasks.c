
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * @file FC_tasks.c															 *
 * @brief																	 *
 * This file contains all task definitions and peripheral initializations	 *
 * required for the flight control system. It initializes peripherals,		 *
 * creates FreeRTOS tasks, and defines the main loop of each task that		 *
 * handles joystick input, IMU processing, PID computation, motor mixing,	 *
 * telemetry, sensor reading, and safety/failsafe mechanisms.				 *
 *																			 *
 * Tasks are created with specific priorities to ensure the real-time		 *
 * constraints of the flight controller, such as achieving the 4 ms control  *
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
#include "uart.h"
#include "telemetry.h"
#include "util.h"

void xHandleRCRxTask			 (void* parameters);
static void xHandleIMUTask	     (void* parameters);
static void xHandleControlTask	 (void* parameters);
static void xHandleSensorsTask	 (void* parameters);
static void xHandleTelemetryTask (void* parameters);
static void xHandleSafetyTask	 (void* parameters);
static void xHandlePIDErrorTask	 (void* parameters);

TaskHandle_t RC_RX_ID, IMU_ID, CTRL_ID, SENSORS_ID, TELEM_ID, SAFETY_ID, PID_ERROR_ID;
PWM_Outputs_t ccr;
QueueHandle_t 	  telem_queue;
QueueHandle_t 	  flight_data_queue;
QueueHandle_t 	  pid_error_queue;
SemaphoreHandle_t MPU_Semaphore;
UserControl_t 	  joystick_data;
static uint8_t    FAILSAFE_ACTIVATED;

/**
 * @brief Initialize all peripherals used by the flight controller.
 *
 * This function configures I2C, SPI, UART1 (with DMA), ADC, PWM,
 * and timers.
 */
void Periph_Init()
{
	__disable_irq();
	I2C1_Init();
	SPI_Init();
	uart1_txrx_init_dma();
	init_delay_timer();
	init_adc();
	pwm_init();
	dt_timer_init();
	__enable_irq();
}

/**
 * @brief Create all FreeRTOS tasks and related synchronization primitives.
 *
 * This function creates tasks for RC input, IMU processing, PID control,
 * sensor reading, telemetry transmission, safety handling, and PID error
 * computation. It also creates queues and semaphores for inter-task communication.
 */
void createTasks()
{
	BaseType_t status;

    /* Create RC receiver task (medium priority) */
	status = xTaskCreate(xHandleRCRxTask,"RC_RX",200,NULL,2,&RC_RX_ID); /* Task 1 */
	configASSERT(status == pdPASS); 

    /* Create IMU processing task (medium priority) */
	status = xTaskCreate(xHandleIMUTask,"IMU",200,NULL,2,&IMU_ID);
	configASSERT(status == pdPASS);

    /* Create Control task for PID computation and motor output (high priority) */
	status = xTaskCreate(xHandleControlTask,"CTRL",200,NULL,5,&CTRL_ID);
	configASSERT(status == pdPASS);

    /* Create Sensors task to read battery and pressure sensors (low priority) */
	status = xTaskCreate(xHandleSensorsTask,"SENSORS",200,NULL,1,&SENSORS_ID);
	configASSERT(status == pdPASS);

    /* Create Telemetry task to send data via UART (lowest priority) */
	status = xTaskCreate(xHandleTelemetryTask,"TELEM",200,NULL,0,&TELEM_ID);
	configASSERT(status == pdPASS);

    /* Create Safety/Failsafe task (high priority) */
	status = xTaskCreate(xHandleSafetyTask,"SAFETY",200,NULL,4,&SAFETY_ID);
	configASSERT(status == pdPASS);

    /* Create PID error computation task (medium-high priority) */
	status = xTaskCreate(xHandlePIDErrorTask,"ERROR",200,NULL,3,&PID_ERROR_ID);
	configASSERT(status == pdPASS);

    /* Create queues for inter-task communication */
	telem_queue = xQueueCreate(6,sizeof(Telemetry_t)); 
	configASSERT(telem_queue != NULL);

	flight_data_queue = xQueueCreate(6,sizeof(FlightMessage_t));
	configASSERT(flight_data_queue != NULL);

	pid_error_queue = xQueueCreate(10,sizeof(FlightMessage_t));
	configASSERT(pid_error_queue != NULL);

    /* Semaphore for IMU access */
	MPU_Semaphore = xSemaphoreCreateBinary();
	configASSERT(MPU_Semaphore != NULL);
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
	setpoint.type = SETPOINT;

	if(!FAILSAFE_ACTIVATED){
		while(1)
		{
            /* Wait for notification to read joystick */
			ulTaskNotifyTake(pdTRUE,portMAX_DELAY);

            /* Read joystick axes */
			get_joystick_data(&joystick_data);

            /* Convert joystick to flight setpoints */
			map_joystick_to_setpoint(joystick_data, &setpoint);

            /* Send setpoint to flight data queue */
			xQueueSend(flight_data_queue,(void *)&setpoint,portMAX_DELAY);

            /* Notify IMU task to proceed */
			xTaskNotifyGive(IMU_ID);

            /* Small delay to allow other tasks to run */
			vTaskDelay(pdMS_TO_TICKS(1));
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
			ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
			map_joystick_to_setpoint(joystick_data, &setpoint);
			xQueueSend(flight_data_queue,(void *)&setpoint,portMAX_DELAY);
			xTaskNotifyGive(IMU_ID);
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
	FlightMessage_t imu_attitude;

    /* Throttle not used for IMU data */
	imu_attitude.throttle = 0;

    /* Initialize MPU sensor */
	init_mpu6050();

	uint32_t lastcall = get_time_now_us();

	while(1)
	{
        /* Read accelerometer and gyroscope */
		mpu_read_acc_gyr(imu_data);

        /* Compute pitch, roll, yaw */
		computeAttitudeFromIMU(imu_data, &imu_attitude.attitude,&lastcall);

        /* Send attitude data */
		xQueueSend(flight_data_queue,(void *)&imu_attitude,portMAX_DELAY);

        /* Notify RC task that IMU data is ready */
		xTaskNotifyGive(RC_RX_ID);

        /* Wait until RC task finishes */
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);

        /* Delay to yield CPU */
		vTaskDelay(pdMS_TO_TICKS(1));
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
	FlightMessage_t 	  pid_error;

	PID_Controller_t roll_pid_params = {.kp = ROLL_KP,
										.ki = ROLL_KI,
										.kd = ROLL_KD,
										.output_min = ROLL_OUTPUT_MIN,
										.output_max = ROLL_OUTPUT_MAX
	};

	PID_Controller_t pitch_pid_params = {.kp = PITCH_KP,
										 .ki = PITCH_KI,
										 .kd = PITCH_KD,
										 .output_min = PITCH_OUTPUT_MIN,
										 .output_max = PITCH_OUTPUT_MAX
	};

	PID_Controller_t yaw_pid_params = {.kp = YAW_KP,
									   .ki = YAW_KI,
									   .kd = YAW_KD,
									   .output_min = YAW_OUTPUT_MIN,
									   .output_max = YAW_OUTPUT_MAX
	};
	PID_Outputs_t output;
	uint32_t lastcall = 0;
	float dt;

	start_PWM();
	lastcall = get_time_now_us();
	while(1)
	{
		xQueueReceive(pid_error_queue,&pid_error,portMAX_DELAY);

		dt = get_time_elapsed(&lastcall);

		output.roll 	= compute_PID(pid_error.attitude.roll, dt, &roll_pid_params);
		output.pitch 	= compute_PID(pid_error.attitude.pitch, dt, &pitch_pid_params);
		output.yaw_rate = compute_PID(pid_error.attitude.yaw_rate, dt, &yaw_pid_params);

		motor_mixer(output,pid_error.throttle,&ccr); 
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

	init_ms5611();

    const TickType_t xFrequency = pdMS_TO_TICKS(200);
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
 *        Puts the MCU into sleep mode to reduce power consumption.
 */void vApplicationIdleHook(void)
{
	 /* Put the MCU into low-power sleep mode until the next interrupt
	       SLEEPDEEP bit is 0, so MCU enters standard sleep (not deep sleep) */
	__WFI();
}



