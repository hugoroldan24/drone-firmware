/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * @file filter.c															   *
 * @brief																	   *
 * Combines accelerometer and gyroscope data to estimate the drone's attitude. *
 * Computes roll and pitch angles using a complementary filter and provides	   *
 * the yaw rate from gyroscope measurements.								   *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <math.h>
#include <stdint.h>
#include "stm32f411xe.h"
#include "const.h"
#include "dron_structs.h"
#include "util.h"

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

