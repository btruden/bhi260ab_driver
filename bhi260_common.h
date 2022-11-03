/*
 * bhi260_common.h
 *
 *  Created on: Sep 10, 2021
 *      Author: btrud
 */

#ifndef INC_IMU_BHI260_COMMON_H_
#define INC_IMU_BHI260_COMMON_H_

#include <stdint.h>
/******************************************************************************
 * Public Constants
 ******************************************************************************/

/******************************************************************************
 * Public Types
 ******************************************************************************/
/**
 * Host command ID variable type
 */
typedef uint16_t bhi260_hostCmd_id_t;

/**
 * Generic return values for bhi260 functions
 */
typedef enum bhi260_ret
{
	BHI260_RET_OK,
	BHI260_RET_FAILED,
	BHI260_RET_WORKING,
	BHI260_RET_IDLE
}bhi260_ret_t;

/**
 * @brief Sensor data types
 * 
 */
typedef enum sensor_data_type
{
	BHI260_SENS_DATA_TYPE_INVALID = 0,
	
	BHI260_SENS_DATA_TYPE_QUATERNION,
	BHI260_SENS_DATA_TYPE_EULER,
	BHI260_SENS_DATA_TYPE_3DVECTOR,
	BHI260_SENS_DATA_TYPE_16BIT_UINT,
	BHI260_SENS_DATA_TYPE_8BIT,
	BHI260_SENS_DATA_TYPE_8BIT_UINT,
	BHI260_SENS_DATA_TYPE_32BIT_UINT,
	BHI260_SENS_DATA_TYPE_16BIT_SINT,
	BHI260_SENS_DATA_TYPE_24BIT_UINT,
	BHI260_SENS_DATA_TYPE_EVENT,
	BHI260_SENS_DATA_TYPE_ACTIVITY,
	BHI260_SENS_DATA_TYPE_8BIT_COUNT_OF_INT,
	BHI260_SENS_DATA_TYPE_STRUCT_NMEA_STRINGS,

	BHI260_SENS_DATA_TYPE_MAX
}sensor_data_type_t;

/**
 * @brief FIFO Event types
 * 
 */
typedef enum event_type
{
	FIFO_EVT_TYPE_VIRTUAL_SENSOR = 0,
	FIFO_EVT_TYPE_DEBUG_DATA,
	FIFO_EVT_TYPE_TIMESTAMP_SMALL_DELTA,
	FIFO_EVT_TYPE_TIMESTAMP_LARGE_DELTA,
	FIFO_EVT_TYPE_TIMESTAMP_FULL,
	FIFO_EVT_TYPE_META_EVENT,
	FIFO_EVT_TYPE_FILLER,
	FIFO_EVT_TYPE_PADDING,

	FIFO_EVT_TYPE_INVALID
}event_type_t;

/**
 * @brief Sensor accuracy levels
 * 
 */
typedef enum sensor_accuracy_status
{
	ACCURACY_UNRELIABLE = 0,
	ACCURACY_LOW,
	ACCURACY_MEDIUM,
	ACCURACY_HIGH
}sensor_accuracy_status_t;

/**
 * @brief Quaternion data structure type
 * 
 */
typedef struct quaternion
{
	double w;
	double x;
	double y;
	double z;
	double accuracy;
}quaternion_t;

/**
 * @brief Euler data structure type
 * 
 */
typedef struct euler
{
	double roll;
	double pitch;
	double yaw;
}euler_t;

/**
 * @brief Gravity vector data type
 * 
 */
typedef struct vector3D
{
	double x;
	double y;
	double z;
}vector3D_t;

/**
 * @brief Gravity data type
 * 
 */
typedef vector3D_t gravity_t;

/******************************************************************************
 * Public Functions
 ******************************************************************************/


#endif /* INC_IMU_BHI260_COMMON_H_ */
