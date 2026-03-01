#include "FreeRTOS.h"
#include "config.h"
#include "state_manager.h"
#include <math.h>

extern QueueHandle_t  flight_data_queue_ID, system_event_queue_ID; 
extern SemaphoreHandle_t IMU_RX_SYN_Semaphore;


/**
 * @brief Task handling IMU measurements.
 *
 * Reads accelerometer and gyroscope data from the MPU6050, computes
 * attitude, sends it to the flight data queue, and notifies the RC task.
 */
void xHandleIMUTask(void* parameters)
{
    float imu_data[6];
	uint32_t lastcall;
	Status_t status = STATUS_OK;
	FlightMessage_t imu_attitude;
	SystemEvent_t event;
	
    DroneState_t system_state;


    /* Throttle not used for IMU data */
	imu_attitude.throttle = 0;

	while(1)
	{
        system_state = get_drone_state();
		switch(system_state)
		{
			case STATE_BOOT:
			    /* Suspend self until activated */
	           vTaskSuspend(NULL);

			   /* Initialize MPU sensor */
	           init_mpu6050();
               event = EVENT_IMU_INITIALIZED;
	           xQueueSend(system_event_queue_ID,(void *)&event,portMAX_DELAY);
			break;

			case STATE_STANDBY:
			   /* Read accelerometer and gyroscope */
	           status = mpu_read_acc_gyr(imu_data); 
			   status = checkIMU(imu_data);

			   if(status == STATUS_OK)
			   {
			      event = EVENT_IMU_VERIFIED;
	              xQueueSend(system_event_queue_ID,(void *)&event,portMAX_DELAY);
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
	   xQueueSend(flight_data_queue_ID,(void *)&imu_attitude,portMAX_DELAY);

       /* Wait until CTRL task sends notification to start again */
	   ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
	}
}


/**
 * @brief Computes the drone's attitude from IMU data.
 *        Uses accelerometer for absolute roll/pitch reference and gyroscope for
 *        rate integration. Applies a complementary filter to combine both sensors.
 *
 * @param imu_data Pointer to an array of 6 floats:
 *                 [0]=acc_x, [1]=acc_y, [2]=acc_z,
 *                 [3]=gyr_x, [4]=gyr_y, [5]=gyr_z
 * @param attitude Pointer to Attitude_t structure to store computed roll, pitch, yaw rate
 * @param lastcall Pointer to previous timestamp for delta time calculation
 */
void computeAttitudeFromIMU(float imu_data[6], Attitude_t* attitude,uint32_t* lastcall)
{
    /* Static variables to keep previous roll and pitch values between calls */
	static float roll;
	static float pitch;

    /* Compute elapsed time since last call in seconds */
	float dt = get_time_elapsed(lastcall);

    /* Calculate pitch and roll from accelerometer (absolute reference) */
	float acc_pitch = atan2f(-imu_data[0], sqrtf(imu_data[1]*imu_data[1] + imu_data[2]*imu_data[2])) * RAD_TO_DEG;
	float acc_roll  = atan2f(imu_data[1], imu_data[2]) * RAD_TO_DEG;

    /* Integrate gyroscope rates to predict new roll and pitch angles */
	roll += imu_data[3] * dt;
	pitch += imu_data[4] * dt;

    /* Yaw rate comes directly from gyroscope z-axis, no integration needed */
	float yaw = imu_data[5];

    /* Apply complementary filter to combine gyro and accelerometer for roll/pitch */
	roll = ALPHA * roll + (1 - ALPHA) * acc_roll;
    pitch = ALPHA * pitch + (1 - ALPHA) * acc_pitch;

    /* Store results in the output structure */
    attitude->roll     = roll;
    attitude->pitch    = pitch;
    attitude->yaw_rate = yaw;
}

/* imu_data[0] -> acc x */
/* imu_data[1] -> acc y */
/* imu_data[2] -> acc z */

/* imu_data[3] -> gyr x */
/* imu_data[4] -> gyr y */
/* imu_data[5] -> gyr z */
static Status_t checkIMU(float imu_data[6U])
{
   uint8_t i;
   uint8_t offset;
   uint8_t tolerance;

   for(i = 0U; i < 6U ;i++)
   {
      tolerance = (i < 3U) ? ACC_TOLERANCE : GYRO_TOLERANCE;
	  offset = (i == 2U) ? GRAVITY : 0U; /* acc z will be aprox 9.81 if at rest*/

	  if(fabs(imu_data[i] - offset) > tolerance)
	  {
	     return STATUS_NOT_OK
	  }
   }	    
   return STATUS_OK;
}
