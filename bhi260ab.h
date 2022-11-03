/*
 * bhi260ab.h
 *
 *  Created on: Aug 27, 2021
 *      Author: btrud
 */

#ifndef INC_IMU_BHI260AB_H_
#define INC_IMU_BHI260AB_H_


/******************************************************************************
 * Public Constants
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "bhi260_common.h"

/******************************************************************************
 * Public Types
 ******************************************************************************/
/**
 * @brief The complete list of the virtual sensors used in the application
 * 
 */
typedef enum vs_list
{
	VS_GAME_ROTATION_VECTOR = 0,
	VS_GRAVITY,
	VS_ACCEL_CORRECTED,
	VS_GYRO_CORRECTED,
	VS_MAG_CORRECTED,

	VS_MAX
}virtual_sensor_list_t;

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/**
 * Module initialization
 */
void BHI260AB_Init();

/**
 * Module tasks
 */
void BHI260AB_Tasks();

/**
 * @brief Returns if the chip is ready and measuring quaternions
 * 
 * @return true 
 * @return false 
 */
bool BHI260AB_isReady();

/**
 * @brief Gets the quaternion from the given virtual sensor list index
 * 
 * @param idx index value of the virtual sensor to get
 * @return quaternion_t last measurement
 */
quaternion_t BHI260AB_GetQuaternion(virtual_sensor_list_t idx);

/**
 * @brief Gets the 3D vector from the given virtual sensor list index
 * 
 * @param idx index value of the virtual sensor to get
 * @return vector3D_t last measurement
 */
vector3D_t BHI260AB_Get3DVector(virtual_sensor_list_t idx);

/**
 * @brief Returns the accuracy status of the virtual sensor 
 * 
 * @param idx index value of the virtual sensor to get
 * @return sensor_accuracy_status_t last measurement
 */
sensor_accuracy_status_t BHI260AB_GetAccuracyStatus(virtual_sensor_list_t idx);

#endif /* INC_IMU_BHI260AB_H_ */
